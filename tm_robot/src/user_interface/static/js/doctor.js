document.addEventListener("DOMContentLoaded", async () => {
  const container = document.getElementById("excelTable");

  let medicineData = [];
  try {
    const res = await fetch("/api/medicine/");
    if (!res.ok) throw new Error("無法取得藥品資料");
    const medicines = await res.json();

    // 將 medicine 資料轉換為預設填入表格資料
    medicineData = medicines.map(m => [
      m.name,
      `${m.amount}mg`,
      "",       // 頻率，醫師輸入
      "口服",   // 預設使用方式
      ""        // 注意事項，自行填寫
    ]);
  } catch (err) {
    alert("藥品資料載入失敗：" + err.message);
    // 如果失敗也讓使用者填空表格
    medicineData = [["", "", "", "", ""]];
  }

  const hot = new Handsontable(container, {
    data: medicineData,
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

  document.getElementById("prescriptionForm").addEventListener("submit", async function (e) {
    e.preventDefault();

    const name = document.getElementById("PatientName").value.trim();
    const id = document.getElementById("id_number").value.trim();
    const sex = document.querySelector('input[name="sex"]:checked')?.value || "Unknown";
    const age = parseInt(document.getElementById("age").value.trim(), 10) || 0;

    const prescriptions = hot.getData()
      .filter(row => row.some(cell => cell !== null && cell.toString().trim() !== ""))
      .map(row => ({
        medicine_name: row[0],
        medicine_dosage: row[1],
        medicine_frequency: row[2],
        medicine_type: row[3],
        thing_of_note: row[4]
      }));

    if (!name || !id || prescriptions.length === 0) {
      alert("請完整填寫所有欄位與至少一筆處方！");
      return;
    }

    const patient = {
      name,
      sex,
      age,
      created_at: new Date().toISOString(),
      picked_state: "尚未開始",
      collected_state: "尚未開始",
      prescriptions
    };

    try {
      const res = await fetch("/api/prescription/register", {
        method: "POST",
        headers: {
          "Content-Type": "application/json"
        },
        body: JSON.stringify(patient)
      });

      if (!res.ok) throw new Error(await res.text());
      const result = await res.json();

      alert("處方已提交！病患 ID：" + result.patient_id);
      this.reset();
      hot.loadData([["", "", "", "", ""]]);
    } catch (err) {
      alert("送出失敗：" + err.message);
      console.error(err);
    }
  });
});
