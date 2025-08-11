// /
//   整合藥物管理系統 JavaScript
//   Integrated Medicine Management System
//  /

// 全域變數
let basicMedicines = [];
let detailedMedicines = [];
let currentEditingMedicine = null;

// API基礎URL
const API_BASE = '/api';

// 初始化
document.addEventListener('DOMContentLoaded', function() {
    initializeSystem();
    setupEventListeners();
    loadAllData();
});

/
  系統初始化
 /
function initializeSystem() {
    console.log(' 整合藥物管理系統初始化...');
    updateStatistics();
}

/
  設置事件監聽器
 /
function setupEventListeners() {
    // 表單提交
    document.getElementById('medicineForm').addEventListener('submit', handleMedicineSubmit);
    
    // 搜尋功能
    document.getElementById('inventorySearch').addEventListener('input', filterInventory);
    document.getElementById('medicineSearch').addEventListener('input', filterMedicineList);
    
    // 篩選功能
    document.getElementById('stockFilter').addEventListener('change', filterInventory);
    document.getElementById('categoryFilter').addEventListener('change', filterMedicineList);
}

/
  標籤切換
 /
function switchTab(tabName) {
    // 移除所有活動狀態
    document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(content => content.classList.remove('active'));
    
    // 設置新的活動標籤
    event.target.classList.add('active');
    document.getElementById(tabName).classList.add('active');
    
    // 根據標籤載入對應數據
    switch(tabName) {
        case 'manage-inventory':
            refreshInventory();
            break;
        case 'medicine-list':
            refreshMedicineList();
            break;
    }
}

/
  載入所有數據
 /
async function loadAllData() {
    try {
        await Promise.all([
            loadBasicMedicines(),
            loadDetailedMedicines()
        ]);
        updateStatistics();
    } catch (error) {
        console.error('載入數據失敗:', error);
        showStatus('載入數據失敗: ' + error.message, 'error');
    }
}

/
  載入基本藥物數據
 /
async function loadBasicMedicines() {
    try {
        const response = await fetch(`${API_BASE}/medicine/basic`);
        const data = await response.json();
        // API直接返回藥物數組，不是 {medicines: [...]} 格式
        basicMedicines = Array.isArray(data) ? data : [];
        return basicMedicines;
    } catch (error) {
        console.error('載入基本藥物失敗:', error);
        throw error;
    }
}

/
  載入詳細藥物數據
 /
async function loadDetailedMedicines() {
    try {
        const response = await fetch(`${API_BASE}/medicine/detailed`);
        const data = await response.json();
        // API直接返回藥物數組，不是 {medicines: [...]} 格式
        detailedMedicines = Array.isArray(data) ? data : [];
        return detailedMedicines;
    } catch (error) {
        console.error('載入詳細藥物失敗:', error);
        throw error;
    }
}



/
  處理藥物表單提交
 /
async function handleMedicineSubmit(event) {
    event.preventDefault();
    
    const formData = new FormData(event.target);
    const statusDiv = document.getElementById('addMedicineStatus');
    
    try {
        // 發送請求
        const response = await fetch(`${API_BASE}/medicine/unified`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                basic: {
                    name: formData.get('name'),
                    amount: parseInt(formData.get('amount')),
                    position: formData.get('position'),
                    manufacturer: "",
                    dosage: ""
                },
                detailed: {
                    description: formData.get('description') || ""
                }
            })
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus(' ' + result.message, 'success', 'addMedicineStatus');
            document.getElementById('medicineForm').reset();
            await loadAllData(); // 重新載入數據
            // 更新當前顯示的標籤
            const activeTab = document.querySelector('.tab.active');
            if (activeTab && activeTab.textContent.includes('新增藥物')) {
                // 如果當前在新增藥物標籤，刷新庫存和藥物列表
                await refreshInventory();
                await refreshMedicineList();
            }
        } else {
            const errorMessage = typeof result.detail === 'string' ? result.detail : 
                               typeof result.detail === 'object' ? JSON.stringify(result.detail) :
                               '保存失敗';
            throw new Error(errorMessage);
        }
        
    } catch (error) {
        const errorMessage = error.message || error.toString() || '未知錯誤';
        showStatus(' 錯誤: ' + errorMessage, 'error', 'addMedicineStatus');
        console.error('藥物提交錯誤:', error);
    }
}

/
  更新統計資訊
 /
function updateStatistics() {
    const totalMedicines = basicMedicines.length;
    const totalStock = basicMedicines.reduce((sum, med) => sum + (med.amount || ), );
    const lowStockCount = basicMedicines.filter(med => (med.amount || ) < ).length;
    const detailedCount = detailedMedicines.length;
    
    document.getElementById('totalMedicines').textContent = totalMedicines;
    document.getElementById('totalStock').textContent = totalStock;
    document.getElementById('lowStockCount').textContent = lowStockCount;
    document.getElementById('detailedCount').textContent = detailedCount;
}

/
  刷新庫存管理
 /
async function refreshInventory() {
    await loadBasicMedicines();
    renderInventoryList();
    updateStatistics();
}

/
  渲染庫存列表
 /
function renderInventoryList(filteredMedicines = null) {
    const container = document.getElementById('inventoryList');
    const medicines = filteredMedicines || basicMedicines;
    
    if (medicines.length === ) {
        container.innerHTML = '<div style="text-align: center; padding: px; color: fcd;"> 暫無庫存資料</div>';
        return;
    }
    
    container.innerHTML = medicines.map(medicine => `
        <div class="medicine-item">
            <div class="medicine-info">
                <div class="medicine-name">${medicine.name}</div>
                <div class="medicine-details">
                    位置: ${medicine.position}
                </div>
            </div>
            <div class="medicine-stock">
                <div class="stock-number ${medicine.amount <  ? 'stock-low' : ''}">${medicine.amount}</div>
                <div style="font-size: px; color: fcd;">庫存</div>
            </div>
            <div class="medicine-actions">
                <button class="btn btn-primary btn-sm" onclick="adjustStock('${medicine.name}', 'add')"></button>
                <button class="btn btn-warning btn-sm" onclick="adjustStock('${medicine.name}', 'subtract')"></button>
                <button class="btn btn-danger btn-sm" onclick="deleteMedicine('${medicine.name}')"></button>
            </div>
        </div>
    `).join('');
}

/
  調整庫存
 /
async function adjustStock(medicineName, action) {
    const amount = prompt(`請輸入要${action === 'add' ? '增加' : '減少'}的數量:`, '');
    if (!amount || isNaN(amount) || parseInt(amount) <= ) {
        alert('請輸入有效的數量');
        return;
    }
    
    try {
        const response = await fetch(`${API_BASE}/medicine/adjust-stock`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                medicine_name: medicineName,
                adjustment: action === 'add' ? parseInt(amount) : -parseInt(amount),
                reason: `${action === 'add' ? '增加' : '減少'}庫存`
            })
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus(` ${medicineName} 庫存已${action === 'add' ? '增加' : '減少'} ${amount}`, 'success', 'inventoryStatus');
            await refreshInventory();
        } else {
            const errorMessage = typeof result.detail === 'string' ? result.detail : 
                               typeof result.detail === 'object' ? JSON.stringify(result.detail) :
                               '庫存調整失敗';
            throw new Error(errorMessage);
        }
        
    } catch (error) {
        const errorMessage = error.message || error.toString() || '庫存調整失敗';
        showStatus(' 庫存調整失敗: ' + errorMessage, 'error', 'inventoryStatus');
        console.error('庫存調整錯誤:', error);
    }
}

/
  刪除藥物
 /
async function deleteMedicine(medicineName) {
    if (!confirm(`確定要刪除藥物 "${medicineName}" 嗎？此操作不可恢復。`)) {
        return;
    }
    
    try {
        const response = await fetch(`${API_BASE}/medicine/${encodeURIComponent(medicineName)}`, {
            method: 'DELETE'
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus(` 藥物 "${medicineName}" 已刪除`, 'success', 'inventoryStatus');
            await loadAllData();
            renderInventoryList();
            renderMedicineList();
        } else {
            const errorMessage = typeof result.detail === 'string' ? result.detail : 
                               typeof result.detail === 'object' ? JSON.stringify(result.detail) :
                               '刪除失敗';
            throw new Error(errorMessage);
        }
        
    } catch (error) {
        const errorMessage = error.message || error.toString() || '刪除失敗';
        showStatus(' 刪除失敗: ' + errorMessage, 'error', 'inventoryStatus');
        console.error('刪除藥物錯誤:', error);
    }
}

/
  刷新藥物清單
 /
async function refreshMedicineList() {
    await Promise.all([loadBasicMedicines(), loadDetailedMedicines()]);
    renderMedicineList();
}

/
  渲染藥物清單
 /
function renderMedicineList(filteredMedicines = null) {
    const container = document.getElementById('medicineListContainer');
    const medicines = filteredMedicines || basicMedicines;
    
    if (medicines.length === ) {
        container.innerHTML = '<div style="text-align: center; padding: px; color: fcd;"> 暫無藥物資料</div>';
        return;
    }
    
    container.innerHTML = medicines.map(medicine => {
        const detailed = detailedMedicines.find(d => d.medicine_name === medicine.name);
        return `
            <div class="medicine-item">
                <div class="medicine-info">
                    <div class="medicine-name">${medicine.name}</div>
                    <div class="medicine-details">
                        分類: ${detailed?.category || '未分類'} | 成分: ${detailed?.ingredient || '未指定'}<br>
                        用法: ${detailed?.usage_method || '請諮詢醫師'} | 劑量: ${detailed?.unit_dose || medicine.dosage || '未指定'}
                    </div>
                </div>
                <div class="medicine-stock">
                    <div class="stock-number ${medicine.amount <  ? 'stock-low' : ''}">${medicine.amount}</div>
                    <div style="font-size: px; color: fcd;">庫存</div>
                </div>
                <div class="medicine-actions">
                    <button class="btn btn-primary btn-sm" onclick="editMedicine('${medicine.name}')"></button>
                    <button class="btn btn-warning btn-sm" onclick="viewMedicineDetails('${medicine.name}')"></button>
                    <button class="btn btn-danger btn-sm" onclick="deleteMedicine('${medicine.name}')"></button>
                </div>
            </div>
        `;
    }).join('');
}

/
  編輯藥物
 /
function editMedicine(medicineName) {
    const medicine = basicMedicines.find(m => m.name === medicineName);
    const detailed = detailedMedicines.find(d => d.medicine_name === medicineName);
    
    if (!medicine) {
        alert('找不到該藥物');
        return;
    }
    
    // 切換到新增藥物標籤
    switchTab('add-medicine');
    
    // 填充表單
    document.getElementById('name').value = medicine.name;
    document.getElementById('amount').value = medicine.amount;
    document.getElementById('position').value = medicine.position;
    document.getElementById('usage_days').value = medicine.usage_days || '';
    document.getElementById('manufacturer').value = medicine.manufacturer || '';
    document.getElementById('dosage').value = medicine.dosage || '';
    
    if (detailed) {
        document.getElementById('ingredient').value = detailed.ingredient || '';
        document.getElementById('category').value = detailed.category || '';
        document.getElementById('description').value = detailed.description || '';
        document.getElementById('usage_method').value = detailed.usage_method || '';
        document.getElementById('unit_dose').value = detailed.unit_dose || '';
        document.getElementById('side_effects').value = detailed.side_effects || '';
        document.getElementById('storage_conditions').value = detailed.storage_conditions || '';
        document.getElementById('expiry_date').value = detailed.expiry_date || '';
        document.getElementById('barcode').value = detailed.barcode || '';
        document.getElementById('appearance_type').value = detailed.appearance_type || '';
        document.getElementById('notes').value = detailed.notes || '';
    }
    
    currentEditingMedicine = medicineName;
    
    // 更改按鈕文字
    const submitBtn = document.querySelector('medicineForm button[type="submit"]');
    submitBtn.textContent = ' 更新藥物資料';
}

/
  查看藥物詳細資訊
 /
function viewMedicineDetails(medicineName) {
    const medicine = basicMedicines.find(m => m.name === medicineName);
    const detailed = detailedMedicines.find(d => d.medicine_name === medicineName);
    
    if (!medicine) {
        alert('找不到該藥物');
        return;
    }
    
    const detailsHTML = `
        <div style="max-width: px; margin:  auto;">
            <h style="color: ce; margin-bottom: px;"> ${medicine.name} - 詳細資訊</h>
            
            <div style="background: fffa; padding: px; border-radius: px; margin-bottom: px;">
                <h style="color: db;">基本資訊</h>
                <p><strong>庫存數量:</strong> ${medicine.amount}</p>
                <p><strong>儲存位置:</strong> ${medicine.position}</p>
                <p><strong>製造商:</strong> ${medicine.manufacturer || '未指定'}</p>
                <p><strong>劑量:</strong> ${medicine.dosage || '未指定'}</p>
                <p><strong>使用天數:</strong> ${medicine.usage_days || '未指定'}</p>
            </div>
            
            ${detailed ? `
                <div style="background: efe; padding: px; border-radius: px;">
                    <h style="color: ae;">詳細資訊</h>
                    <p><strong>分類:</strong> ${detailed.category || '未分類'}</p>
                    <p><strong>主要成分:</strong> ${detailed.ingredient || '未指定'}</p>
                    <p><strong>描述:</strong> ${detailed.description || '無'}</p>
                    <p><strong>使用方法:</strong> ${detailed.usage_method || '請諮詢醫師'}</p>
                    <p><strong>單位劑量:</strong> ${detailed.unit_dose || '未指定'}</p>
                    <p><strong>副作用:</strong> ${detailed.side_effects || '無記錄'}</p>
                    <p><strong>儲存條件:</strong> ${detailed.storage_conditions || '常溫'}</p>
                    <p><strong>有效期限:</strong> ${detailed.expiry_date || '未指定'}</p>
                    <p><strong>外觀:</strong> ${detailed.appearance_type || '未描述'}</p>
                    <p><strong>備註:</strong> ${detailed.notes || '無'}</p>
                </div>
            ` : '<div style="background: fffcd; padding: px; border-radius: px; color: ;"> 尚未建立詳細資料</div>'}
        </div>
    `;
    
    // 創建模態框
    showModal(detailsHTML);
}








// 篩選功能
function filterInventory() {
  const searchTerm = (document.getElementById('inventorySearch')?.value || '')
    .trim()
    .toLowerCase();
  const stockFilter = (document.getElementById('stockFilter')?.value || '');

  const filtered = (basicMedicines || []).filter(medicine => {
    const name = (medicine?.name || '').toString().toLowerCase();
    const matchesSearch = !searchTerm || name.includes(searchTerm);

    const amount = Number.parseInt(medicine?.amount ?? 0) || 0;
    let matchesStock = true;

    switch (stockFilter) {
      case 'sufficient':
        // 充足：> 50
        matchesStock = amount > 50;
        break;
      case 'medium':
        // 中等：20 ~ 50
        matchesStock = amount >= 20 && amount <= 50;
        break;
      case 'low':
        // 低量：1 ~ 19
        matchesStock = amount >= 1 && amount < 20;
        break;
      case 'empty':
        // 無庫存：0
        matchesStock = amount === 0;
        break;
      // '', 'all' 或其他值 -> 不過濾庫存
      default:
        matchesStock = true;
    }

    return matchesSearch && matchesStock;
  });

  renderInventoryList(filtered);
}


function filterMedicineList() {
    const searchTerm = document.getElementById('medicineSearch').value.toLowerCase();
    const categoryFilter = document.getElementById('categoryFilter').value;
    
    let filtered = basicMedicines.filter(medicine => {
        const matchesSearch = medicine.name.toLowerCase().includes(searchTerm);
        let matchesCategory = true;
        
        if (categoryFilter) {
            const detailed = detailedMedicines.find(d => d.medicine_name === medicine.name);
            matchesCategory = detailed?.category === categoryFilter;
        }
        
        return matchesSearch && matchesCategory;
    });
    
    renderMedicineList(filtered);
}



/
  顯示狀態訊息
 /
function showStatus(message, type = 'success', elementId = 'addMedicineStatus') {
    const statusDiv = document.getElementById(elementId);
    statusDiv.className = `status-message status-${type}`;
    statusDiv.textContent = message;
    statusDiv.style.display = 'block';
    
    // 秒後自動隱藏
    setTimeout(() => {
        statusDiv.style.display = 'none';
    }, );
}

/
  顯示模態框
 /
function showModal(content) {
    // 創建模態框
    const modal = document.createElement('div');
    modal.style.cssText = `
        position: fixed;
        top: ;
        left: ;
        width: %;
        height: %;
        background: rgba(,,,.);
        display: flex;
        justify-content: center;
        align-items: center;
        z-index: ;
    `;
    
    const modalContent = document.createElement('div');
    modalContent.style.cssText = `
        background: white;
        padding: px;
        border-radius: px;
        max-width: %;
        max-height: %;
        overflow-y: auto;
        position: relative;
    `;
    
    modalContent.innerHTML = content + `
        <button onclick="this.closest('.modal').remove()" 
                style="position: absolute; top: px; right: px; 
                       background: ecc; color: white; border: none; 
                       border-radius: %; width: px; height: px; 
                       cursor: pointer;"></button>
    `;
    
    modal.className = 'modal';
    modal.appendChild(modalContent);
    document.body.appendChild(modal);
    
    // 點擊背景關閉
    modal.addEventListener('click', (e) => {
        if (e.target === modal) {
            modal.remove();
        }
    });
}

// 工具函數
function formatDate(dateString) {
    if (!dateString) return '未指定';
    return new Date(dateString).toLocaleDateString('zh-TW');
}

function formatDateTime(dateString) {
    if (!dateString) return '未記錄';
    return new Date(dateString).toLocaleString('zh-TW');
}