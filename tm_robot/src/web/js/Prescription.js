const statusOptions = ["尚未開始", "準備中", "完成"];

let patientList = [];

function fetchPatientListFromBackend() {
  // patientList = [
  //   {
  //     name: "John Doe",
  //     id: "A123456789",
  //     sex: "Male",
      
  //     prescription: [
  //       ["Paracetamol", "500mL", "3x/day"],
  //       ["Ibuprofen", "200mL", "2x/day"],
  //       ["Cough Syrup", "1tablets", "2x/day"]
  //     ],
  //     picked_state: "尚未開始",
  //     collected_state: "尚未領取",
  //     created_at: "2024-05-01 10:03"
  //   },
  //   {
  //     name: "Jane Smith",
  //     id: "B987654321",
  //     sex: "Female",
  //     prescription: [
  //       ["Amoxicillin", "250mL", "every 8h"],
  //       ["Vitamin D", "1000", "1x/day"],
  //       ["Antihistamine", "10mL", "1x/day"]
  //     ],
  //     picked_state: "準備中",
  //     collected_state: "尚未領取",
  //     created_at: "2024-05-02 12:45"
  //   },
  //   {
  //     name: "Alice Chen",
  //     id: "C135792468",
  //     sex: "Female",
      
  //     prescription: [
  //       ["Vitamin C", "100mL", "1x/day"],
  //       ["Zinc", "15mL", "1x/day"],
  //       ["Probiotic", "1 cap", "1x/day"]
  //     ],
  //     picked_state: "完成",
  //     collected_state: "已領取",
  //     created_at: "2024-05-03 15:10"
  //   }
  // ];

  renderTable();
}

function renderTable() {
  const tbody = document.querySelector("#patientTable tbody");
  tbody.innerHTML = "";

  patientList.forEach((p) => {
    const tr = document.createElement("tr");

    const prescriptionDetails = `
      <details>
        <summary>查看處方內容</summary>
        <ul>
          ${p.prescription.map(
            d => `<li>${d[0]} - ${d[1]} - ${d[2]}</li>`
          ).join("")}
        </ul>
      </details>
    `;

    tr.innerHTML = `
      <td>${p.name}</td>
      <td>${p.id}</td>
      <td>${p.sex}</td>
      <td>${prescriptionDetails}</td>
      <td>${p.picked_state}</td>
      <td>${p.collected_state}</td>
      <td>${p.created_at}</td>
    `;

    tbody.appendChild(tr);
  });
}



function showDetails(index) {
  const p = patientList[index];
  const msg = `${p.name} 的處方內容：\n` + p.prescription.map(
    d => `- ${d[0]} (${d[1]}, ${d[2]})`
  ).join("\n");
  alert(msg);
}

document.addEventListener("DOMContentLoaded", fetchPatientListFromBackend);
