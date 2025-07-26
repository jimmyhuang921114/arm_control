document.addEventListener("DOMContentLoaded", async () => {
  const container = document.getElementById('medicineTable');

  let data = [];
  try {
    const res = await fetch('/api/medicine/');
    if (!res.ok) throw new Error("無法取得藥品資料");
    const medicines = await res.json();

    data = medicines.map(m => [
      m.name,
      `${m.amount} tablets`,
      m.expiration_date || 'YYYY-MM-DD', 
      m.position
    ]);
  } catch (err) {
    alert("藥品載入失敗：" + err.message);
    data = [["", "", "", ""]];
  }

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
