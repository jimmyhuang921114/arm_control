//  JavaScript
class EnhancedMedicineManager {
    constructor() {
        this.apiBase = 'http://localhost:/api';
        this.hot = null;
        this.basicData = [];
        this.currentEditingMedicine = null;
        this.init();
    }

    async init() {
        await this.loadBasicMedicines();
        this.initBasicTable();
        this.initDetailedMedicineForm();
        this.setupEventListeners();
        await this.loadMedicineSelect();
    }

    // ====================  ====================

    async loadBasicMedicines() {
        try {
            const response = await fetch(`${this.apiBase}/medicine/`);
            if (response.ok) {
                const medicines = await response.json();
                this.basicData = medicines.map(med => [
                    med.name,
                    med.amount,
                    med.usage_days,
                    med.position,
                    med.id
                ]);
            } else {
                console.error('Failed to load basic medicines:', response.statusText);
                this.basicData = [];
            }
        } catch (error) {
            console.error('Error loading basic medicines:', error);
            this.basicData = [];
        }
    }

    initBasicTable() {
        const container = document.getElementById('medicineTable');
        this.hot = new Handsontable(container, {
            data: this.basicData,
            colHeaders: ['', '', '', '', ''],
            columns: [
                { type: 'text', placeholder: '' },
                { type: 'numeric', placeholder: '' },
                { type: 'numeric', placeholder: '' },
                { type: 'text', placeholder: ' A-' },
                { type: 'text', readOnly: true, renderer: this.actionButtonRenderer.bind(this) }
            ],
            stretchH: 'all',
            rowHeaders: true,
            minSpareRows: 1,
            height: 400,
            licenseKey: 'non-commercial-and-evaluation',
            afterChange: this.handleCellChange.bind(this),
            contextMenu: ['remove_row'],
            afterRemoveRow: this.handleRowDelete.bind(this)
        });
    }

    actionButtonRenderer(instance, td, row, col, prop, value, cellProperties) {
        td.innerHTML = '';
        if (row < this.basicData.length && this.basicData[row].length >= ) {
            const saveBtn = document.createElement('button');
            saveBtn.textContent = '';
            saveBtn.className = 'action-btn save-btn';
            saveBtn.onclick = () => this.saveBasicMedicine(row);

            const deleteBtn = document.createElement('button');
            deleteBtn.textContent = '';
            deleteBtn.className = 'action-btn delete-btn';
            deleteBtn.onclick = () => this.deleteBasicMedicine(row);

            const viewBtn = document.createElement('button');
            viewBtn.textContent = '';
            viewBtn.className = 'action-btn btn-info';
            viewBtn.onclick = () => this.viewMedicineDetails(this.basicData[row][]);

            td.appendChild(saveBtn);
            td.appendChild(deleteBtn);
            td.appendChild(viewBtn);
        } else {
            const addBtn = document.createElement('button');
            addBtn.textContent = '';
            addBtn.className = 'action-btn add-btn';
            addBtn.onclick = () => this.addBasicMedicine(row);
            td.appendChild(addBtn);
        }
        return td;
    }

    async handleCellChange(changes, source) {
        if (!changes || source === 'loadData') return;

        for (const change of changes) {
            const [row, col, oldValue, newValue] = change;
            if (this.basicData[row] && this.basicData[row].length >=  && this.basicData[row][]) {
                clearTimeout(this.saveTimeout);
                this.saveTimeout = setTimeout(() => {
                    this.saveBasicMedicine(row);
                }, );
            }
        }
    }

    async addBasicMedicine(row) {
        const rowData = this.hot.getDataAtRow(row);
        const [name, amount, usage_days, position] = rowData;

        if (!name || !amount || !usage_days || !position) {
            this.showMessage('', 'error');
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: name,
                    amount: parseInt(amount),
                    usage_days: parseInt(usage_days),
                    position: position
                })
            });

            if (response.ok) {
                const newMedicine = await response.json();
                this.basicData[row] = [name, amount, usage_days, position, newMedicine.id];
                this.hot.render();
                this.showMessage('', 'success');
                await this.loadMedicineSelect(); //
            } else {
                const error = await response.json();
                this.showMessage(`: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error adding medicine:', error);
            this.showMessage('', 'error');
        }
    }

    async saveBasicMedicine(row) {
        const rowData = this.hot.getDataAtRow(row);
        const [name, amount, usage_days, position, id] = rowData;

        if (!id) {
            await this.addBasicMedicine(row);
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/${id}`, {
                method: 'PUT',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: name,
                    amount: parseInt(amount),
                    usage_days: parseInt(usage_days),
                    position: position
                })
            });

            if (response.ok) {
                this.showMessage('', 'success');
            } else {
                const error = await response.json();
                this.showMessage(`: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error updating medicine:', error);
            this.showMessage('', 'error');
        }
    }

    async deleteBasicMedicine(row) {
        const rowData = this.hot.getDataAtRow(row);
        const id = rowData[];

        if (!id) {
            this.hot.alter('remove_row', row);
            return;
        }

        if (!confirm('')) {
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/${id}`, {
                method: 'DELETE'
            });

            if (response.ok) {
                this.hot.alter('remove_row', row);
                this.basicData.splice(row, );
                this.showMessage('', 'success');
                await this.loadMedicineSelect(); //
            } else {
                const error = await response.json();
                this.showMessage(`: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error deleting medicine:', error);
            this.showMessage('', 'error');
        }
    }

    // ====================  ====================

    initDetailedMedicineForm() {
        //
        this.clearDetailedForm();
    }

    async loadMedicineSelect() {
        const select = document.getElementById('medicineSelect');
        select.innerHTML = '<option value="">--  --</option>';

        try {
            const response = await fetch(`${this.apiBase}/medicine/`);
            if (response.ok) {
                const medicines = await response.json();
                medicines.forEach(medicine => {
                    const option = document.createElement('option');
                    option.value = medicine.name;
                    option.textContent = `${medicine.name} (: ${medicine.position})`;
                    select.appendChild(option);
                });
            }
        } catch (error) {
            console.error('Error loading medicine select:', error);
        }
    }

    async loadDetailedMedicineInfo(medicineName) {
        if (!medicineName) {
            this.clearDetailedForm();
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/detailed/${encodeURIComponent(medicineName)}`);
            if (response.ok) {
                const data = await response.json();
                this.populateDetailedForm(medicineName, data);
                this.currentEditingMedicine = medicineName;
                document.getElementById('updateDetailedInfo').style.display = 'inline-block';
                document.getElementById('saveDetailedInfo').style.display = 'none';
            } else {
                //
                this.clearDetailedForm();
                document.getElementById('medicineName').value = medicineName;
                this.currentEditingMedicine = null;
                document.getElementById('updateDetailedInfo').style.display = 'none';
                document.getElementById('saveDetailedInfo').style.display = 'inline-block';
                this.showMessage('', 'info');
            }
        } catch (error) {
            console.error('Error loading detailed medicine info:', error);
            this.showMessage('', 'error');
        }
    }

    populateDetailedForm(medicineName, data) {
        document.getElementById('medicineName').value = data?.medicine_name || medicineName;
        document.getElementById('dosage').value = data?.dosage || '';
        document.getElementById('manufacturer').value = data?.manufacturer || '';
        document.getElementById('usageMethod').value = data?.usage_method || '';
        document.getElementById('unitDose').value = data?.unit_dose || '';

        document.getElementById('color').value = data?.color || '';
        document.getElementById('shape').value = data?.shape || '';

        document.getElementById('expiryDate').value = data?.expiry_date || '';
        document.getElementById('barcode').value = data?.barcode || '';
        document.getElementById('companyFullName').value = data?.company_full_name || '';
        document.getElementById('drugFullName').value = data?.drug_full_name || '';

        document.getElementById('code1').value = data?.code1 || '';
        document.getElementById('code2').value = data?.code2 || '';
        document.getElementById('code3').value = data?.code3 || '';

        document.getElementById('indications').value = data?.indications || '';
        document.getElementById('sideEffects').value = data?.side_effects || '';
        document.getElementById('usage').value = data?.usage || '';
        document.getElementById('precautions').value = data?.precautions || '';
        document.getElementById('pregnancyClass').value = data?.pregnancy_class || '';
        document.getElementById('storageConditions').value = data?.storage_conditions || '';
    }

    clearDetailedForm() {
        const inputs = [
            'medicineName', 'dosage', 'manufacturer', 'usageMethod', 'unitDose',
            'color', 'shape', 'expiryDate', 'barcode', 'companyFullName', 'drugFullName',
            'code', 'code', 'code', 'indications', 'sideEffects', 'usage',
            'precautions', 'pregnancyClass', 'storageConditions'
        ];

        inputs.forEach(id => {
            const element = document.getElementById(id);
            if (element) element.value = '';
        });

        this.currentEditingMedicine = null;
        document.getElementById('updateDetailedInfo').style.display = 'none';
        document.getElementById('saveDetailedInfo').style.display = 'inline-block';
    }

    getDetailedFormData() {
    return {
        basic_info: {
            medicine_name: document.getElementById('medicineName').value,
            dosage: document.getElementById('dosage').value,
            manufacturer: document.getElementById('manufacturer').value,
            usage_method: document.getElementById('usageMethod').value,
            unit_dose: document.getElementById('unitDose').value
        },
        appearance: {
            color: document.getElementById('color').value,
            shape: document.getElementById('shape').value
        },
        packaging: {
            expiry_date: document.getElementById('expiryDate').value,
            barcode: document.getElementById('barcode').value,
            company_full_name: document.getElementById('companyFullName').value,
            drug_full_name: document.getElementById('drugFullName').value
        },
        codes: {
            code1: document.getElementById('code1').value,
            code2: document.getElementById('code2').value,
            code3: document.getElementById('code3').value
        },
        indications: document.getElementById('indications').value,
        side_effects: document.getElementById('sideEffects').value,
        usage: document.getElementById('usage').value,
        precautions: document.getElementById('precautions').value,
        pregnancy_class: document.getElementById('pregnancyClass').value,
        storage_conditions: document.getElementById('storageConditions').value
    };
}


    async saveDetailedMedicineInfo() {
        const medicineName = document.getElementById('medicineName').value;
        if (!medicineName) {
            this.showMessage('', 'error');
            return;
        }

        const medicineData = this.getDetailedFormData();

        try {
            const response = await fetch(`${this.apiBase}/medicine/detailed/`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    medicine_name: medicineName,
                    medicine_data: medicineData
                })
            });

            if (response.ok) {
                this.showMessage('', 'success');
                this.currentEditingMedicine = medicineName;
                document.getElementById('updateDetailedInfo').style.display = 'inline-block';
                document.getElementById('saveDetailedInfo').style.display = 'none';
            } else {
                const error = await response.json();
                this.showMessage(`: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error saving detailed medicine info:', error);
            this.showMessage('', 'error');
        }
    }

    // ====================  ====================

    async searchDetailedMedicine(searchTerm, searchType = 'name') {
        if (!searchTerm.trim()) {
            this.showMessage('', 'error');
            return;
        }

        try {
            let endpoint;
            switch (searchType) {
                case 'code':
                    endpoint = `/medicine/search/code/${encodeURIComponent(searchTerm)}`;
                    break;
                case 'name':
                default:
                    endpoint = `/medicine/integrated/${encodeURIComponent(searchTerm)}`;
                    break;
            }

            const response = await fetch(`${this.apiBase}${endpoint}`);
            if (response.ok) {
                const data = await response.json();
                this.displayMedicineInfo(data);
            } else {
                document.getElementById('medicineInfoDisplay').innerHTML =
                    '<p class="placeholder-text"></p>';
                this.showMessage('', 'error');
            }
        } catch (error) {
            console.error('Error searching medicine:', error);
            this.showMessage('', 'error');
        }
    }

    displayMedicineInfo(data) {
        const display = document.getElementById('medicineInfoDisplay');

        if (data.medicine_name) {
            //
            display.innerHTML = this.formatIntegratedMedicineInfo(data);
        } else {
            //
            display.innerHTML = this.formatSearchResults(data);
        }
    }

    formatIntegratedMedicineInfo(data) {
        let html = `<div class="medicine-info">
            <h>${data.medicine_name}</h>`;

        //
        if (data.basic_info) {
            html += `<div class="info-section inventory-info">
                <h></h>
                <div class="info-item">
                    <span class="info-label"></span>
                    <span class="info-value">${data.basic_info.amount}</span>
                </div>
                <div class="info-item">
                    <span class="info-label"></span>
                    <span class="info-value">${data.basic_info.position}</span>
                </div>
                <div class="info-item">
                    <span class="info-label"></span>
                    <span class="info-value">${data.basic_info.usage_days}</span>
                </div>
            </div>`;
        }

        //
        if (data.detailed_info) {
            const d = data.detailed_info;

            // 小工具：把物件渲染成一個區塊（只顯示有值的欄位）
            const renderSection = (title, obj) => {
                if (!obj || typeof obj !== 'object') return '';
                const rows = Object.entries(obj)
                    .filter(([_, v]) => v !== undefined && v !== null && String(v).trim() !== '')
                    .map(([k, v]) => `
                        <div class="info-item">
                            <span class="info-label">${k}</span>
                            <span class="info-value">${v}</span>
                        </div>
                    `).join('');
                return rows ? `<div class="info-section detailed-info"><h3>${title}</h3>${rows}</div>` : '';
            };

            // 分組（物件）
            html += renderSection('外觀', d.appearance);  // e.g. { color, shape, ... }
            html += renderSection('包裝', d.packaging);   // e.g. { unit_dose, expiry_date, ... }
            html += renderSection('代碼', d.codes);       // e.g. { barcode, nh_code, atc_code, ... }

            // 重要單欄位（字串）
            const importantFields = [
                ['適應症', d.indications],
                ['副作用', d.side_effects],
                ['用法用量', d.usage],
                ['注意事項', d.precautions],
                ['孕期分級', d.pregnancy_class],
                ['儲存條件', d.storage_conditions],
            ];

            importantFields.forEach(([label, value]) => {
                if (value !== undefined && value !== null && String(value).trim() !== '') {
                    html += `
                        <div class="info-section detailed-info">
                            <h3>${label}</h3>
                            <div class="info-item">
                                <span class="info-value">${value}</span>
                            </div>
                        </div>
                    `;
                }
            });
        }

        // 原本結尾保留
        html += '</div>';
        return html;

    }

    formatSearchResults(results) {
        let html = '<div class="medicine-info"><h></h>';

        Object.entries(results).forEach(([name, data]) => {
            html += `<div class="info-section">
                <h>${name}</h>`;

            if (data.matched_code) {
                html += `<div class="info-item">
                    <span class="info-label"></span>
                    <span class="info-value">${data.matched_code.type} - ${data.matched_code.value}</span>
                </div>`;
            }

            html += `<button class="btn-info" onclick="medicineManager.viewMedicineDetails('${name}')">

            </button></div>`;
        });

        html += '</div>';
        return html;
    }

    async viewMedicineDetails(medicineName) {
        await this.searchDetailedMedicine(medicineName, 'name');
    }

    // ==================== JSON  ====================

    showJSONPreview() {
        const medicineData = this.getDetailedFormData();
        const medicineName = document.getElementById('medicineName').value;

        if (!medicineName) {
            this.showMessage('請先輸入藥品名稱', 'error');
            return;
        }

        const jsonData = {
            medicine_name: medicineName,
            ...medicineData
        };

        document.getElementById('jsonPreview').textContent = JSON.stringify(jsonData, null, 2);
        document.getElementById('jsonModal').style.display = 'block';
    }


    copyJSON() {
        const jsonText = document.getElementById('jsonPreview').textContent;
        navigator.clipboard.writeText(jsonText).then(() => {
            this.showMessage('JSON', 'success');
        }).catch(err => {
            console.error('Failed to copy JSON:', err);
            this.showMessage('', 'error');
        });
    }

    downloadJSON() {
        const jsonText = document.getElementById('jsonPreview').textContent;
        const medicineName = document.getElementById('medicineName').value || 'medicine';

        const blob = new Blob([jsonText], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${medicineName}_${new Date().toISOString().split('T')[]}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        this.showMessage('JSON', 'success');
    }

    async exportDetailedJSON() {
        try {
            const response = await fetch(`${this.apiBase}/export/medicines/integrated`);
            if (response.ok) {
                const data = await response.json();

                const blob = new Blob([JSON.stringify(data, null, )], { type: 'application/json' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = `detailed_medicines_${new Date().toISOString().split('T')[]}.json`;
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                URL.revokeObjectURL(url);

                this.showMessage('', 'success');
            } else {
                this.showMessage('', 'error');
            }
        } catch (error) {
            console.error('Error exporting detailed JSON:', error);
            this.showMessage('', 'error');
        }
    }

    // ====================  ====================

    setupEventListeners() {
        //
        document.getElementById('medicineSelect').addEventListener('change', (e) => {
            this.loadDetailedMedicineInfo(e.target.value);
        });

        //
        document.getElementById('loadDetailedInfo').addEventListener('click', () => {
            const selectedMedicine = document.getElementById('medicineSelect').value;
            if (selectedMedicine) {
                this.loadDetailedMedicineInfo(selectedMedicine);
            } else {
                this.showMessage('', 'error');
            }
        });

        document.getElementById('clearForm').addEventListener('click', () => {
            this.clearDetailedForm();
        });

        document.getElementById('saveDetailedInfo').addEventListener('click', () => {
            this.saveDetailedMedicineInfo();
        });

        document.getElementById('updateDetailedInfo').addEventListener('click', () => {
            this.saveDetailedMedicineInfo();
        });

        document.getElementById('viewDetailedJSON').addEventListener('click', () => {
            this.showJSONPreview();
        });

        document.getElementById('exportDetailedJSON').addEventListener('click', () => {
            this.exportDetailedJSON();
        });

        //
        document.getElementById('searchDetailedBtn').addEventListener('click', () => {
            const searchTerm = document.getElementById('searchMedicineInput').value;
            this.searchDetailedMedicine(searchTerm, 'name');
        });

        document.getElementById('searchByCodeBtn').addEventListener('click', () => {
            const searchTerm = document.getElementById('searchMedicineInput').value;
            this.searchDetailedMedicine(searchTerm, 'code');
        });

        document.getElementById('searchMedicineInput').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                this.searchDetailedMedicine(e.target.value, 'name');
            }
        });

        //
        document.querySelector('.close').addEventListener('click', () => {
            document.getElementById('jsonModal').style.display = 'none';
        });

        document.getElementById('copyJSON').addEventListener('click', () => {
            this.copyJSON();
        });

        document.getElementById('downloadJSON').addEventListener('click', () => {
            this.downloadJSON();
        });

        //
        window.addEventListener('click', (e) => {
            const modal = document.getElementById('jsonModal');
            if (e.target === modal) {
                modal.style.display = 'none';
            }
        });

        //
        this.addControlButtons();
    }

    addControlButtons() {
        const header = document.querySelector('.header');
        if (!header) return;

        const buttonContainer = document.createElement('div');
        buttonContainer.className = 'control-buttons';

        //
        const reloadBtn = this.createButton('', async () => {
            await this.loadBasicMedicines();
            this.hot.loadData(this.basicData);
            await this.loadMedicineSelect();
            this.showMessage('', 'success');
        });

        //
        const exportBasicBtn = this.createButton('', () => {
            this.exportBasicJSON();
        });

        buttonContainer.appendChild(reloadBtn);
        buttonContainer.appendChild(exportBasicBtn);
        header.appendChild(buttonContainer);
    }

    createButton(text, onClick) {
        const button = document.createElement('button');
        button.textContent = text;
        button.className = 'btn-secondary';
        button.addEventListener('click', onClick);
        return button;
    }

    exportBasicJSON() {
        // 只輸出有資料的列（name 或 id 有其一即可）
        const jsonData = this.basicData
            .filter(row => Array.isArray(row) && row.length >= 4 && (row[0] || row[4]))
            .map(row => ({
                name: (row[0] || '').toString().trim(),
                amount: Number.parseInt(row[1] ?? 0) || 0,
                usage_days: Number.parseInt(row[2] ?? 0) || 0,
                position: (row[3] || '').toString().trim(),
                id: row[4] ?? null
            }));

        const blob = new Blob([JSON.stringify(jsonData, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `basic_medicines_${new Date().toISOString().split('T')[0]}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        this.showMessage('基本藥品清單已匯出', 'success');
    }


    // ====================  ====================

    showMessage(message, type = 'info') {
        let messageDiv = document.getElementById('message');
        if (!messageDiv) {
            messageDiv = document.createElement('div');
            messageDiv.id = 'message';
            messageDiv.style.cssText = `
                position: fixed;
                top: px;
                right: px;
                padding: px px;
                border-radius: px;
                color: white;
                z-index: ;
                transition: opacity .s;
                font-weight: ;
                box-shadow:  px px rgba(,,,.);
            `;
            document.body.appendChild(messageDiv);
        }

        messageDiv.textContent = message;
        messageDiv.className = `message-${type}`;

        const colors = {
            success: 'ae',
            error: 'ecc',
            info: 'db'
        };
        messageDiv.style.backgroundColor = colors[type] || colors.info;
        messageDiv.style.opacity = '';

        setTimeout(() => {
            if (messageDiv) {
                messageDiv.style.opacity = '';
                setTimeout(() => {
                    if (messageDiv && messageDiv.parentNode) {
                        messageDiv.parentNode.removeChild(messageDiv);
                    }
                }, );
            }
        }, );
    }

    async handleRowDelete(index, amount) {
        for (let i = ; i < amount; i++) {
            const rowIndex = index + i;
            if (this.basicData[rowIndex] && this.basicData[rowIndex][]) {
                await this.deleteMedicineById(this.basicData[rowIndex][]);
            }
        }
        await this.loadMedicineSelect();
    }

    async deleteMedicineById(id) {
        try {
            await fetch(`${this.apiBase}/medicine/${id}`, { method: 'DELETE' });
        } catch (error) {
            console.error('Error deleting medicine by ID:', error);
        }
    }
}

//
let medicineManager;

document.addEventListener("DOMContentLoaded", () => {
    medicineManager = new EnhancedMedicineManager();
});