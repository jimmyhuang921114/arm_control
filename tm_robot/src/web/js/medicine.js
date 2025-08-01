document.addEventListener("DOMContentLoaded", () => {
  // const data = [
  //   ['Amoxicillin', '30 tablets', '2025-12-31', 'A1-01'],
  //   ['Paracetamol', '20 tablets', '2025-10-15', 'A2-03'],
  //   ['Cough Syrup', '1 bottle (100ml)', '2025-08-31', 'B1-02'],
  // ];

  const container = document.getElementById('medicineTable');
  const hot = new Handsontable(container, {
    data: data,
    colHeaders: ['藥品名稱', '藥物數量', '有效日期', '藥物位置'],
    columns: [
      { type: 'text', placeholder: '輸入藥名' },
      { type: 'text', placeholder: '輸入數量' },
      { type: 'date', dateFormat: 'YYYY-MM-DD', correctFormat: true, placeholder: 'YYYY-MM-DD' },
      { type: 'text', placeholder: '儲位位置（如 A1-01）' }
    ],
    stretchH: 'all',
    rowHeaders: true,
    minSpareRows: 1, 
    height: 300,
    licenseKey: 'non-commercial-and-evaluation'
  });
});
