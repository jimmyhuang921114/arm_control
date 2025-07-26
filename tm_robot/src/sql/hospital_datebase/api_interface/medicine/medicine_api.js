const apiBase = "/api/medicine/";

async function addMedicine() {
  const data = {
    name: document.getElementById("name").value,
    usage: document.getElementById("usage").value,
    stock: parseInt(document.getElementById("stock").value)
  };

  const res = await fetch(apiBase, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(data)
  });

  alert(res.ok ? "✅ 新增成功" : "❌ 新增失敗");
}

async function listMedicines() {
  const res = await fetch(apiBase);
  const data = await res.json();
  document.getElementById("medicineList").innerText = JSON.stringify(data, null, 2);
}

async function searchMedicine() {
  const name = document.getElementById("searchName").value;
  const res = await fetch(apiBase + name);
  const data = await res.json();
  document.getElementById("searchResult").innerText = JSON.stringify(data, null, 2);
}

async function deleteMedicine() {
  const id = document.getElementById("deleteId").value;
  const res = await fetch(apiBase + id, { method: "DELETE" });
  const result = await res.json();
  document.getElementById("deleteResult").innerText = JSON.stringify(result, null, 2);
}
