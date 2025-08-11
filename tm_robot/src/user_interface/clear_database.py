#!/usr/bin/env python3
"""
Database Cleaner Tool
Clear all test data from the hospital management system database
"""

import os
import sqlite3
from pathlib import Path

def clear_database(db_path):
    """Clear all data from the database"""
    try:
        if not os.path.exists(db_path):
            print(f"❌ Database file not found: {db_path}")
            return False
        
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Get all table names
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = cursor.fetchall()
        
        if not tables:
            print(f"ℹ️ No tables found in {db_path}")
            conn.close()
            return True
        
        print(f"🗑️ Clearing data from {db_path}...")
        
        # Disable foreign key constraints temporarily
        cursor.execute("PRAGMA foreign_keys = OFF;")
        
        # Clear all tables
        for table in tables:
            table_name = table[0]
            if table_name != 'sqlite_sequence':  # Skip system table
                cursor.execute(f"DELETE FROM {table_name};")
                print(f"   ✅ Cleared table: {table_name}")
        
        # Reset auto-increment counters
        cursor.execute("DELETE FROM sqlite_sequence;")
        
        # Re-enable foreign key constraints
        cursor.execute("PRAGMA foreign_keys = ON;")
        
        conn.commit()
        conn.close()
        
        print(f"✅ Database {db_path} cleared successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Error clearing database {db_path}: {e}")
        return False

def delete_database_file(db_path):
    """Delete the entire database file"""
    try:
        if os.path.exists(db_path):
            os.remove(db_path)
            print(f"🗑️ Database file deleted: {db_path}")
            return True
        else:
            print(f"ℹ️ Database file not found: {db_path}")
            return True
    except Exception as e:
        print(f"❌ Error deleting database file {db_path}: {e}")
        return False

def main():
    """Main function"""
    print("🏥 Hospital Database Cleaner")
    print("=" * 40)
    
    # List of possible database files
    db_files = [
        "complete_hospital_medicine.db",
        "hospital_medicine.db",
        "hospital_medicine_stable.db",
        "hospital_medicine_improved.db",
        "hospital_medicine_final.db",
        "hospital_medicine_working.db"
    ]
    
    found_databases = []
    for db_file in db_files:
        if os.path.exists(db_file):
            found_databases.append(db_file)
    
    if not found_databases:
        print("ℹ️ No database files found in current directory")
        return
    
    print(f"📋 Found {len(found_databases)} database file(s):")
    for i, db in enumerate(found_databases, 1):
        file_size = os.path.getsize(db) / 1024  # Size in KB
        print(f"   {i}. {db} ({file_size:.1f} KB)")
    
    print("\n🔧 Choose action:")
    print("   1. Clear data (keep structure)")
    print("   2. Delete files completely")
    print("   3. Cancel")
    
    try:
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == "1":
            # Clear data from all databases
            print(f"\n🗑️ Clearing data from {len(found_databases)} database(s)...")
            success_count = 0
            for db in found_databases:
                if clear_database(db):
                    success_count += 1
            
            print(f"\n✅ Successfully cleared {success_count}/{len(found_databases)} databases")
            
        elif choice == "2":
            # Delete database files
            print(f"\n🗑️ Deleting {len(found_databases)} database file(s)...")
            confirm = input("⚠️ This will permanently delete all database files. Continue? (y/N): ").strip().lower()
            
            if confirm == 'y':
                success_count = 0
                for db in found_databases:
                    if delete_database_file(db):
                        success_count += 1
                
                print(f"\n✅ Successfully deleted {success_count}/{len(found_databases)} database files")
            else:
                print("❌ Operation cancelled")
                
        elif choice == "3":
            print("❌ Operation cancelled")
            
        else:
            print("❌ Invalid choice")
            
    except KeyboardInterrupt:
        print("\n❌ Operation cancelled by user")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()