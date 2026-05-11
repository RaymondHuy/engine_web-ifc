import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const DEFAULT_API_BASE = (() => {
  const fromQuery = new URLSearchParams(window.location.search).get("api");
  if (fromQuery) {
    return fromQuery;
  }

  const host = window.location.hostname || "127.0.0.1";
  return `${window.location.protocol === "file:" ? "http:" : window.location.protocol}//${host}:18080`;
})();

const INITIAL_FILE_PATH = new URLSearchParams(window.location.search).get("filePath");

const els = {
  apiBase: document.getElementById("apiBase"),
  clearAllButton: document.getElementById("clearAllButton"),
  clearButton: document.getElementById("clearButton"),
  connectionStatus: document.getElementById("connectionStatus"),
  connectionText: document.getElementById("connectionText"),
  dropMeta: document.getElementById("dropMeta"),
  dropTitle: document.getElementById("dropTitle"),
  dropzone: document.getElementById("dropzone"),
  fileInput: document.getElementById("fileInput"),
  fileName: document.getElementById("fileName"),
  fileSize: document.getElementById("fileSize"),
  fileStrip: document.getElementById("fileStrip"),
  healthButton: document.getElementById("healthButton"),
  progressText: document.getElementById("progressText"),
  progressWrap: document.getElementById("progressWrap"),
  requestState: document.getElementById("requestState"),
  resetViewButton: document.getElementById("resetViewButton"),
  responseOutput: document.getElementById("responseOutput"),
  summaryFile: document.getElementById("summaryFile"),
  summaryPath: document.getElementById("summaryPath"),
  summaryStatus: document.getElementById("summaryStatus"),
  uploadButton: document.getElementById("uploadButton"),
  uploadProgress: document.getElementById("uploadProgress"),
  viewer3d: document.getElementById("viewer3d"),
  viewerEmpty: document.getElementById("viewerEmpty"),
};

let selectedFile = null;
let busy = false;
let scene3d;
let camera;
let renderer;
let controls;
const modelGroups = [];
const loadedModels = [];
let lastCameraTarget = null;

function normalizeBaseUrl(value) {
  return value.trim().replace(/\/+$/, "");
}

function formatBytes(bytes) {
  if (!Number.isFinite(bytes)) {
    return "-";
  }

  const units = ["B", "KB", "MB", "GB"];
  let size = bytes;
  let unit = 0;
  while (size >= 1024 && unit < units.length - 1) {
    size /= 1024;
    unit++;
  }

  return `${size.toFixed(unit === 0 ? 0 : 1)} ${units[unit]}`;
}

function formatNumber(value) {
  if (!Number.isFinite(Number(value))) {
    return "-";
  }

  return new Intl.NumberFormat().format(Number(value));
}

function setConnection(state, text) {
  els.connectionStatus.dataset.state = state;
  els.connectionText.textContent = text;
}

function setBusy(nextBusy) {
  busy = nextBusy;
  els.uploadButton.disabled = busy || !selectedFile;
  els.healthButton.disabled = busy;
  els.fileInput.disabled = busy;
  els.clearButton.disabled = busy;
  els.clearAllButton.disabled = busy;
}

function setProgress(value) {
  const rounded = Math.max(0, Math.min(100, Math.round(value)));
  els.uploadProgress.value = rounded;
  els.progressText.textContent = `${rounded}%`;
}

function compactPayload(payload) {
  if (!payload || typeof payload !== "object") {
    return payload;
  }

  if (Array.isArray(payload.geometries)) {
    const { geometries, ...rest } = payload;
    return {
      ...rest,
      geometries: `${geometries.length} geometry buffers omitted`,
    };
  }

  return payload;
}

function showResponse(payload) {
  els.responseOutput.textContent = JSON.stringify(compactPayload(payload), null, 2);
}

function setSelectedFile(file) {
  selectedFile = file;
  els.uploadButton.disabled = busy || !selectedFile;

  if (!file) {
    els.fileInput.value = "";
    els.fileStrip.hidden = true;
    els.dropTitle.textContent = "Choose IFC file";
    els.dropMeta.textContent = ".ifc, .ifczip, .ifcxml";
    return;
  }

  els.fileName.textContent = file.name;
  els.fileSize.textContent = formatBytes(file.size);
  els.fileStrip.hidden = false;
  els.dropTitle.textContent = file.name;
  els.dropMeta.textContent = formatBytes(file.size);
}

function setResultIdle() {
  els.requestState.textContent = "Idle";
  els.summaryStatus.textContent = "No request";
  els.summaryFile.textContent = "-";
  els.summaryPath.textContent = "-";
  els.viewerEmpty.textContent = "No model";
  els.progressWrap.hidden = true;
  setProgress(0);
  showResponse({});
  clearModel();
}

function removeFilePathFromUrl() {
  const url = new URL(window.location.href);
  if (!url.searchParams.has("filePath")) {
    return;
  }

  url.searchParams.delete("filePath");
  window.history.replaceState({}, "", `${url.pathname}${url.search}${url.hash}`);
}

function clearAll() {
  setSelectedFile(null);
  setResultIdle();
  removeFilePathFromUrl();
}

function prepareNextUpload() {
  setSelectedFile(null);
  els.progressWrap.hidden = true;
  setProgress(0);
  removeFilePathFromUrl();
}

function getLoadedTotals() {
  return loadedModels.reduce(
    (totals, model) => ({
      modelCount: totals.modelCount + 1,
      meshCount: totals.meshCount + (model.meshCount || 0),
      triangleCount: totals.triangleCount + (model.triangleCount || 0),
      vertexCount: totals.vertexCount + (model.vertexCount || 0),
    }),
    { modelCount: 0, meshCount: 0, triangleCount: 0, vertexCount: 0 },
  );
}

function setResultFromGeometry(payload, uploadPayload) {
  els.requestState.textContent = payload && payload.ok ? "Rendered" : "Failed";
  const totals = getLoadedTotals();
  const modelLabel = totals.modelCount === 1 ? "1 model" : `${totals.modelCount} models`;
  els.summaryStatus.textContent = payload && payload.ok ? modelLabel : "Error";
  els.summaryFile.textContent = payload && payload.ok ? formatNumber(totals.meshCount) : "-";
  els.summaryPath.textContent = payload && payload.ok ? formatNumber(totals.triangleCount) : "-";
  showResponse({
    loadedModels,
    totals,
    upload: uploadPayload,
    geometry: compactPayload(payload),
  });
}

async function checkHealth() {
  const base = normalizeBaseUrl(els.apiBase.value);
  if (!base) {
    setConnection("offline", "Missing API URL");
    return false;
  }

  localStorage.setItem("viewerCppApiBase", base);
  setConnection("checking", "Checking API");

  try {
    const response = await fetch(`${base}/health`, { method: "GET" });
    const payload = await response.json().catch(() => null);
    if (!response.ok || !payload || payload.ok !== true) {
      setConnection("offline", "API error");
      return false;
    }

    setConnection("online", "API online");
    return true;
  } catch (error) {
    setConnection("offline", "API offline");
    return false;
  }
}

function uploadToApi(file) {
  return new Promise((resolve, reject) => {
    const base = normalizeBaseUrl(els.apiBase.value);
    const data = new FormData();
    data.append("file", file, file.name);

    const xhr = new XMLHttpRequest();
    xhr.open("POST", `${base}/api/ifc/upload`);
    xhr.responseType = "text";

    xhr.upload.addEventListener("progress", (event) => {
      if (event.lengthComputable) {
        setProgress((event.loaded / event.total) * 100);
      }
    });

    xhr.addEventListener("load", () => {
      const payload = safeJsonParse(xhr.responseText) || {};
      if (xhr.status >= 200 && xhr.status < 300) {
        resolve(payload);
      } else {
        reject(new Error(payload.error || `Upload failed with HTTP ${xhr.status}`));
      }
    });

    xhr.addEventListener("error", () => {
      reject(new Error("Network error while uploading to the C++ API"));
    });

    xhr.send(data);
  });
}

async function fetchGeometry(filePath) {
  const base = normalizeBaseUrl(els.apiBase.value);
  const url = `${base}/api/ifc/geometry?filePath=${encodeURIComponent(filePath)}`;
  const response = await fetch(url);
  const payload = await response.json().catch(() => null);

  if (!response.ok || !payload) {
    throw new Error(payload && payload.error ? payload.error : `Geometry failed with HTTP ${response.status}`);
  }

  if (payload.ok !== true) {
    throw new Error(payload.error || "Geometry endpoint returned an error");
  }

  return payload;
}

function safeJsonParse(text) {
  try {
    return JSON.parse(text);
  } catch {
    return null;
  }
}

async function uploadSelectedFile() {
  if (!selectedFile || busy) {
    return;
  }

  setBusy(true);
  els.progressWrap.hidden = false;
  els.requestState.textContent = "Uploading";
  els.summaryStatus.textContent = "Uploading";
  els.summaryFile.textContent = "-";
  els.summaryPath.textContent = "-";
  setProgress(0);

  try {
    await checkHealth();
    const uploadPayload = await uploadToApi(selectedFile);
    const firstFile = uploadPayload && Array.isArray(uploadPayload.files) ? uploadPayload.files[0] : null;

    setProgress(100);
    els.requestState.textContent = "Loading";
    els.summaryStatus.textContent = "Loading geometry";
    showResponse(uploadPayload);

    if (!firstFile || !firstFile.path) {
      throw new Error("Upload response did not include a saved file path");
    }

    const geometryPayload = await fetchGeometry(firstFile.path);
    renderGeometryPayload(geometryPayload, firstFile);
    setResultFromGeometry(geometryPayload, uploadPayload);
    prepareNextUpload();
  } catch (error) {
    const payload = { ok: false, error: error.message };
    els.requestState.textContent = "Failed";
    els.summaryStatus.textContent = "Error";
    els.summaryFile.textContent = "-";
    els.summaryPath.textContent = "-";
    showResponse(payload);
  } finally {
    setBusy(false);
  }
}

async function loadServerFile(filePath) {
  setBusy(true);
  els.requestState.textContent = "Loading";
  els.summaryStatus.textContent = "Loading geometry";
  els.summaryFile.textContent = "-";
  els.summaryPath.textContent = "-";

  try {
    await checkHealth();
    const geometryPayload = await fetchGeometry(filePath);
    renderGeometryPayload(geometryPayload, { filePath });
    setResultFromGeometry(geometryPayload, { filePath });
  } catch (error) {
    const payload = { ok: false, error: error.message };
    els.requestState.textContent = "Failed";
    els.summaryStatus.textContent = "Error";
    showResponse(payload);
  } finally {
    setBusy(false);
  }
}

function initViewer() {
  scene3d = new THREE.Scene();
  scene3d.background = new THREE.Color(0xe8eef3);

  camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100000);
  camera.position.set(8, 7, 8);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  els.viewer3d.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  const ambientLight = new THREE.AmbientLight(0xffffff, 0.35);
  scene3d.add(ambientLight);

  const directionalLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight1.position.set(1, 2, 1);
  scene3d.add(directionalLight1);

  const directionalLight2 = new THREE.DirectionalLight(0xd8eef5, 0.65);
  directionalLight2.position.set(-2, 1, -1);
  scene3d.add(directionalLight2);

  const grid = new THREE.GridHelper(20, 20, 0x8ca3b5, 0xc7d3dd);
  grid.name = "viewer-grid";
  scene3d.add(grid);

  const resizeObserver = new ResizeObserver(resizeViewer);
  resizeObserver.observe(els.viewer3d);
  resizeViewer();
  animate();
}

function resizeViewer() {
  if (!renderer || !camera) {
    return;
  }

  const width = Math.max(1, els.viewer3d.clientWidth);
  const height = Math.max(1, els.viewer3d.clientHeight);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene3d, camera);
}

function clearModel() {
  if (!scene3d || modelGroups.length === 0) {
    if (els.viewerEmpty) {
      els.viewerEmpty.hidden = false;
    }
    return;
  }

  for (const group of modelGroups) {
    scene3d.remove(group);
    group.traverse((child) => {
      if (child.geometry) {
        child.geometry.dispose();
      }

      if (child.material) {
        if (Array.isArray(child.material)) {
          child.material.forEach((material) => material.dispose());
        } else {
          child.material.dispose();
        }
      }
    });
  }

  modelGroups.length = 0;
  loadedModels.length = 0;
  lastCameraTarget = null;
  els.viewerEmpty.hidden = false;
}

function renderGeometryPayload(payload, source = {}) {
  const parts = Array.isArray(payload.geometries) ? payload.geometries : [];
  if (parts.length === 0) {
    if (modelGroups.length === 0) {
      els.viewerEmpty.textContent = "No geometry";
      els.viewerEmpty.hidden = false;
    }
    return;
  }

  els.viewerEmpty.hidden = true;
  const modelGroup = new THREE.Group();
  const modelName = source.originalName || source.filePath || payload.filePath || `Model ${modelGroups.length + 1}`;
  modelGroup.name = modelName;

  for (const part of parts) {
    const mesh = createMeshFromPart(part);
    if (mesh) {
      modelGroup.add(mesh);
    }
  }

  scene3d.add(modelGroup);
  modelGroups.push(modelGroup);
  loadedModels.push({
    name: modelName,
    filePath: payload.filePath,
    schema: payload.schema,
    meshCount: payload.meshCount || 0,
    geometryCount: payload.geometryCount || 0,
    vertexCount: payload.vertexCount || 0,
    triangleCount: payload.triangleCount || 0,
  });
  fitCameraToModel();
}

function createMeshFromPart(part) {
  if (!Array.isArray(part.vertexData) || !Array.isArray(part.indexData) || part.vertexData.length < 6) {
    return null;
  }

  const vertexCount = Math.floor(part.vertexData.length / 6);
  const positions = new Float32Array(vertexCount * 3);
  const normals = new Float32Array(vertexCount * 3);
  const colors = new Float32Array(vertexCount * 3);
  const color = Array.isArray(part.color) ? part.color : [1, 1, 1, 1];

  for (let source = 0, target = 0; source < vertexCount * 6; source += 6, target += 3) {
    positions[target + 0] = part.vertexData[source + 0];
    positions[target + 1] = part.vertexData[source + 1];
    positions[target + 2] = part.vertexData[source + 2];
    normals[target + 0] = part.vertexData[source + 3];
    normals[target + 1] = part.vertexData[source + 4];
    normals[target + 2] = part.vertexData[source + 5];
    colors[target + 0] = color[0] ?? 1;
    colors[target + 1] = color[1] ?? 1;
    colors[target + 2] = color[2] ?? 1;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(part.indexData), 1));

  const alpha = color[3] ?? 1;
  const material = new THREE.MeshPhongMaterial({
    side: THREE.DoubleSide,
    vertexColors: true,
    transparent: alpha < 1,
    opacity: alpha < 1 ? alpha : 1,
  });

  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = `${part.type || "IFC"} #${part.expressID || ""}`.trim();

  if (Array.isArray(part.transform) && part.transform.length === 16) {
    mesh.matrix.fromArray(part.transform);
    mesh.matrixAutoUpdate = false;
  }

  return mesh;
}

function fitCameraToModel() {
  if (modelGroups.length === 0) {
    return;
  }

  const box = new THREE.Box3();
  for (const group of modelGroups) {
    group.updateMatrixWorld(true);
    box.expandByObject(group);
  }
  if (box.isEmpty()) {
    return;
  }

  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 1);
  const fov = THREE.MathUtils.degToRad(camera.fov);
  const distance = Math.abs(maxDim / Math.sin(fov / 2)) * 0.75;
  const direction = new THREE.Vector3(1, 0.8, 1).normalize();

  camera.near = Math.max(distance / 1000, 0.01);
  camera.far = Math.max(distance * 100, 1000);
  camera.position.copy(center).add(direction.multiplyScalar(distance));
  camera.updateProjectionMatrix();

  controls.target.copy(center);
  controls.update();
  lastCameraTarget = center.clone();
}

function resetView() {
  if (modelGroups.length > 0) {
    fitCameraToModel();
    return;
  }

  camera.position.set(8, 7, 8);
  controls.target.set(0, 0, 0);
  controls.update();
}

function pickFirstFile(fileList) {
  return fileList && fileList.length > 0 ? fileList[0] : null;
}

function wireEvents() {
  els.apiBase.addEventListener("change", checkHealth);
  els.healthButton.addEventListener("click", checkHealth);
  els.resetViewButton.addEventListener("click", resetView);
  els.uploadButton.addEventListener("click", uploadSelectedFile);
  els.clearButton.addEventListener("click", clearAll);
  els.clearAllButton.addEventListener("click", clearAll);

  els.fileInput.addEventListener("change", () => {
    setSelectedFile(pickFirstFile(els.fileInput.files));
  });

  ["dragenter", "dragover"].forEach((eventName) => {
    els.dropzone.addEventListener(eventName, (event) => {
      event.preventDefault();
      els.dropzone.classList.add("is-dragging");
    });
  });

  ["dragleave", "drop"].forEach((eventName) => {
    els.dropzone.addEventListener(eventName, (event) => {
      event.preventDefault();
      els.dropzone.classList.remove("is-dragging");
    });
  });

  els.dropzone.addEventListener("drop", (event) => {
    setSelectedFile(pickFirstFile(event.dataTransfer.files));
  });
}

function init() {
  els.apiBase.value = localStorage.getItem("viewerCppApiBase") || DEFAULT_API_BASE;
  initViewer();
  setResultIdle();
  wireEvents();
  checkHealth().then(() => {
    if (INITIAL_FILE_PATH) {
      loadServerFile(INITIAL_FILE_PATH);
    }
  });
}

init();
