// 後端 API：請改成你的實際端點
const API_URL = '/api/patient_list';

let patientList = [];

// --- 新增：排序用小工具 ---
function toInt(v, def = 9999) {
  const n = parseInt(v, 10);
  return Number.isFinite(n) ? n : def;
}
function normCabinet(x) {
  if (typeof x === 'number') return ['', x];
  if (typeof x === 'string') {
    const head = (x.match(/[A-Za-z]+/g) || []).join('').toUpperCase();
    const num = toInt((x.match(/\d+/g) || [0])[0], 0);
    return [head, num];
  }
  return ['', 0];
}
function parseLocateEntry(entry) {
  // entry 可能是: [name, unit, freq, locate?]
  const loc = entry?.[3];
  let row = 9999, col = 9999, shelf = 9999;
  let [cabA, cabN] = ['', 0];

  if (Array.isArray(loc) && loc.length >= 2) {
    row = toInt(loc[0], 9999);
    col = toInt(loc[1], 9999);
  } else if (typeof loc === 'string') {
    const parts = loc.replace(/-/g, ',').split(',');
    if (parts.length >= 2) {
      row = toInt(parts[parts.length - 2], 9999);
      col = toInt(parts[parts.length - 1], 9999);
    }
    [cabA, cabN] = normCabinet(loc);
  } else if (loc && typeof loc === 'object') {
    if (Array.isArray(loc.locate)) {
      row = toInt(loc.locate[0], 9999);
      col = toInt(loc.locate[1], 9999);
    }
    shelf = toInt(loc.shelf, 9999);
    [cabA, cabN] = normCabinet(loc.cabinet);
    row = toInt(loc.row, row);
    col = toInt(loc.col, col);
  }
  const name = entry?.[0] || '';
  return { cabA, cabN, shelf, row, col, name };
}
function comparePrescription(a, b) {
  const A = parseLocateEntry(a);
  const B = parseLocateEntry(b);
  return (
    A.cabA.localeCompare(B.cabA) ||
    A.cabN - B.cabN ||
    A.shelf - B.shelf ||
    A.row - B.row ||
    A.col - B.col ||
    A.name.localeCompare(B.name)
  );
}

// --- 修改：只替換這個函數 ---
async function fetchPatientListFromBackend() {
  try {
    const res = await fetch(API_URL, { headers: { 'Accept': 'application/json' } });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    patientList = await res.json();   // 期待回傳為陣列

    // ✅ 這裡對每位病患的處方依「櫃→層→row→col→name」排序
    patientList.forEach(p => {
      if (Array.isArray(p?.prescription)) {
        p.prescription.sort(comparePrescription);
      }
    });

    renderTable();
  } catch (err) {
    console.error('載入病患清單失敗:', err);
    const tbody = document.querySelector('#patientTable tbody');
    if (tbody) {
      tbody.innerHTML = `
        <tr><td colspan="8" style="color:#c00;">載入失敗：${String(err)}</td></tr>
      `;
    }
  }
}


function renderTable() {
  const tbody = document.querySelector('#patientTable tbody');
  if (!tbody) return;

  if (!Array.isArray(patientList) || patientList.length === 0) {
    tbody.innerHTML = `<tr><td colspan="8" style="text-align:center;">尚無資料</td></tr>`;
    return;
  }

  // 先清空
  tbody.innerHTML = '';

  patientList.forEach((p, i) => {
    const tr = document.createElement('tr');

    // 建立 <details> 處方清單（用 DOM API 避免 innerHTML 的 XSS 風險）
    const details = document.createElement('details');
    const summary = document.createElement('summary');
    summary.textContent = '查看處方詳細';
    details.appendChild(summary);

    const ul = document.createElement('ul');
    const pres = Array.isArray(p.prescription) ? p.prescription : [];
    pres.forEach(d => {
      const li = document.createElement('li');
      const name = (d && d[0]) ? d[0] : '';
      const unit = (d && d[1]) ? d[1] : '';
      const freq = (d && d[2]) ? d[2] : '';
      li.textContent = `${name} - ${unit} - ${freq}`;
      ul.appendChild(li);
    });
    details.appendChild(ul);

    // 逐欄位塞入
    tr.appendChild(tdText(p?.name ?? ''));
    tr.appendChild(tdText(p?.id ?? ''));
    tr.appendChild(tdText(p?.sex ?? ''));
    tr.appendChild(tdNode(details));
    tr.appendChild(tdText(p?.picked_state ?? ''));
    tr.appendChild(tdText(p?.collected_state ?? ''));
    tr.appendChild(tdText(p?.created_at ?? ''));

    // 操作：詳細按鈕（選用）
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.textContent = '詳細';
    btn.onclick = () => showDetails(i);
    tr.appendChild(tdNode(btn));

    tbody.appendChild(tr);
  });
}

function tdText(text) {
  const td = document.createElement('td');
  td.textContent = text;
  return td;
}

function tdNode(node) {
  const td = document.createElement('td');
  td.appendChild(node);
  return td;
}

function showDetails(index) {
  const p = patientList[index];
  const pres = Array.isArray(p?.prescription) ? p.prescription : [];
  const lines = pres.map(d => {
    const name = (d && d[0]) ? d[0] : '';
    const unit = (d && d[1]) ? d[1] : '';
    const freq = (d && d[2]) ? d[2] : '';
    return `- ${name} (${unit}, ${freq})`;
  }).join('\n');
  alert(`${p?.name ?? ''}\n${lines}`);
}

document.addEventListener('DOMContentLoaded', fetchPatientListFromBackend);
