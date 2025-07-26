document.addEventListener("DOMContentLoaded", () => {
  const container = document.getElementById("excelTable");

  const hot = new Handsontable(container, {
    // data: [
    //   ["Paracetamol", "500mg", "3x/day", "口服", "飯後服用"],
    //   ["Ibuprofen", "200mg", "2x/day", "口服", "避免空腹"],
    //   ["Paracetamol", "30tablets", "3x/day", "口服", "飯後服用"],
    // ],
    colHeaders: ["藥物名稱", "總劑量", "使用頻率", "使用方式", "注意事項"],
    columns: [
      { type: 'text' },
      { type: 'text' },
      { type: 'text' },
      { type: 'text' },
      { type: 'text' }
    ],
    rowHeaders: true,
    minSpareRows: 1, 
    stretchH: 'all',
    height: 300,
    licenseKey: 'non-commercial-and-evaluation'
  });

  document.getElementById("prescriptionForm").addEventListener("submit", function (e) {
    e.preventDefault();

    const name = document.getElementById("PatientName").value.trim();
    const id = document.getElementById("id_number").value.trim();
    const sex = document.querySelector('input[name="sex"]:checked')?.value || "Unknown";

    const prescription = hot.getData().filter(row =>
      row.some(cell => cell !== null && cell.toString().trim() !== "")
    );

    if (!name || !id || prescription.length === 0) {
      alert("請完整填寫所有欄位與至少一筆處方！");
      return;
    }

    const patient = {
      name,
      id,
      sex,
      created_at: new Date().toLocaleString(),
      prescription,
      picked_state: "尚未開始",
      collected_state: "尚未開始"
    };

    console.log("Submitted Patient:", patient);
    alert("處方已提交！\n請查看 console 以確認送出資料。");

    // 清除表單內容與表格
    this.reset();
    hot.loadData([
      ["", "", "", "", ""]
    ]);
  });
});
