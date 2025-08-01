const statusOptions = ["尚未開始", "準備中", "完成"];

let patientList = [];

function fetchPatientListFromBackend() {
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
