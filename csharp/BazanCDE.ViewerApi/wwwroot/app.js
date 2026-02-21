import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

const viewport = document.getElementById("viewport");
const summaryText = document.getElementById("summaryText");
const loadSampleBtn = document.getElementById("loadSampleBtn");
const uploadInput = document.getElementById("uploadInput");
const uploadAppendInput = document.getElementById("uploadAppendInput");
const clearBtn = document.getElementById("clearBtn");

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);
viewport.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xc8d6e5);

const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 2000);
camera.position.set(16, 12, 16);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const directionalLight = new THREE.DirectionalLight(0xffffff, 0.85);
directionalLight.position.set(8, 16, 12);
scene.add(directionalLight);

const ifcApi = new WebIFC.IfcAPI();
ifcApi.SetWasmPath("https://unpkg.com/web-ifc@0.0.75/", true);
await ifcApi.Init();

let loadedModels = [];

function resize() {
  const width = viewport.clientWidth;
  const height = viewport.clientHeight;
  renderer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

window.addEventListener("resize", resize);
resize();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

animate();

function clearAllModels() {
  for (const model of loadedModels) {
    scene.remove(model.root);
    model.root.traverse((object) => {
      if (!object.isMesh) {
        return;
      }

      object.geometry?.dispose?.();
      if (Array.isArray(object.material)) {
        object.material.forEach((material) => material?.dispose?.());
      } else {
        object.material?.dispose?.();
      }
    });

    ifcApi.CloseModel(model.modelId);
  }

  loadedModels = [];
}

function fitCameraToLoadedModels() {
  if (loadedModels.length === 0) {
    return;
  }

  const bounds = new THREE.Box3();
  for (const model of loadedModels) {
    bounds.expandByObject(model.root);
  }

  if (bounds.isEmpty()) {
    return;
  }

  const center = bounds.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.set(center.x + 16, center.y + 12, center.z + 16);
  controls.update();
}

function updateSummary(summary, sourceLabel, append) {
  const block = `${formatSummary(summary, sourceLabel)}\nLoaded models: ${loadedModels.length}`;
  const hasPrevious = summaryText.textContent &&
    summaryText.textContent !== "No model loaded." &&
    summaryText.textContent !== "Scene cleared.";

  if (append && hasPrevious) {
    summaryText.textContent = `${summaryText.textContent}\n\n${block}`;
    return;
  }

  summaryText.textContent = block;
}

function formatSummary(summary, source) {
  const rows = summary.TopTypes
    .map((x) => `${x.TypeName.padEnd(32, " ")} ${String(x.Count).padStart(6, " ")}`)
    .join("\n");

  return [
    `Source: ${source}`,
    `Schema: ${summary.Schema}`,
    `Data lines: ${summary.DataLineCount}`,
    `Max express id: #${summary.MaxExpressId}`,
    "",
    "Top entity types:",
    rows || "(none)"
  ].join("\n");
}

function toBufferGeometry(vertexData, indexData, color) {
  const geometry = new THREE.BufferGeometry();
  const positions = new Float32Array(vertexData.length / 2);
  const normals = new Float32Array(vertexData.length / 2);
  const colors = new Float32Array(vertexData.length / 2);

  for (let i = 0; i < vertexData.length; i += 6) {
    const dst = i / 2;
    positions[dst + 0] = vertexData[i + 0];
    positions[dst + 1] = vertexData[i + 1];
    positions[dst + 2] = vertexData[i + 2];

    normals[dst + 0] = vertexData[i + 3];
    normals[dst + 1] = vertexData[i + 4];
    normals[dst + 2] = vertexData[i + 5];

    colors[dst + 0] = color.x;
    colors[dst + 1] = color.y;
    colors[dst + 2] = color.z;
  }

  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.setIndex(new THREE.BufferAttribute(indexData, 1));
  return geometry;
}

function createMaterial(placedGeometry) {
  const mat = new THREE.MeshPhongMaterial({
    side: THREE.DoubleSide,
    vertexColors: true,
    transparent: placedGeometry.color.w !== 1
  });

  if (mat.transparent) {
    mat.opacity = placedGeometry.color.w;
  }

  return mat;
}

async function fetchSummary(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Summary request failed (${response.status}).`);
  }

  return response.json();
}

async function loadModelFromApi(ifcUrl, summaryUrl, sourceLabel, append = false) {
  if (!append) {
    clearAllModels();
    summaryText.textContent = "Loading model from API...";
  }

  const modelResponse = await fetch(ifcUrl);
  if (!modelResponse.ok) {
    throw new Error(`Model request failed (${modelResponse.status}).`);
  }

  const data = new Uint8Array(await modelResponse.arrayBuffer());
  const modelId = ifcApi.OpenModel(data, {
    COORDINATE_TO_ORIGIN: true,
    CIRCLE_SEGMENTS: 12
  });

  const modelGroup = new THREE.Group();

  ifcApi.StreamAllMeshes(modelId, (flatMesh) => {
    const placedGeometries = flatMesh.geometries;
    for (let i = 0; i < placedGeometries.size(); i++) {
      const placedGeometry = placedGeometries.get(i);
      const geomData = ifcApi.GetGeometry(modelId, placedGeometry.geometryExpressID);
      const verts = ifcApi.GetVertexArray(geomData.GetVertexData(), geomData.GetVertexDataSize());
      const indices = ifcApi.GetIndexArray(geomData.GetIndexData(), geomData.GetIndexDataSize());

      const geometry = toBufferGeometry(verts, indices, placedGeometry.color);
      const material = createMaterial(placedGeometry);
      const mesh = new THREE.Mesh(geometry, material);

      const matrix = new THREE.Matrix4().fromArray(placedGeometry.flatTransformation);
      mesh.applyMatrix4(matrix);
      mesh.matrixAutoUpdate = false;

      modelGroup.add(mesh);

      if (typeof geomData.delete === "function") {
        geomData.delete();
      }
    }
  });

  scene.add(modelGroup);
  loadedModels.push({ modelId, root: modelGroup });
  fitCameraToLoadedModels();

  const summary = await fetchSummary(summaryUrl);
  updateSummary(summary, sourceLabel, append);
}

async function uploadAndLoad(file, append = false) {
  if (!append || loadedModels.length === 0) {
    const modeText = append ? "Appending IFC via C# API..." : "Uploading IFC to C# API...";
    summaryText.textContent = modeText;
  }

  const bytes = await file.arrayBuffer();
  const uploadResponse = await fetch(`/api/models/upload?fileName=${encodeURIComponent(file.name)}`, {
    method: "POST",
    headers: {
      "Content-Type": "application/octet-stream"
    },
    body: bytes
  });

  if (!uploadResponse.ok) {
    throw new Error(`Upload failed (${uploadResponse.status}).`);
  }

  const uploadResult = await uploadResponse.json();
  const modelId = uploadResult.modelId;

  await loadModelFromApi(
    `/api/models/${modelId}/ifc`,
    `/api/models/${modelId}/summary`,
    `upload:${file.name}`,
    append
  );
}

loadSampleBtn.addEventListener("click", async () => {
  try {
    await loadModelFromApi("/api/models/sample", "/api/models/sample/summary", "sample");
  } catch (error) {
    summaryText.textContent = `Load sample failed: ${error.message}`;
  }
});

uploadInput.addEventListener("change", async (event) => {
  const file = event.target.files?.[0];
  if (!file) {
    return;
  }

  try {
    await uploadAndLoad(file, false);
  } catch (error) {
    summaryText.textContent = `Upload failed: ${error.message}`;
  } finally {
    uploadInput.value = "";
  }
});

uploadAppendInput.addEventListener("change", async (event) => {
  const file = event.target.files?.[0];
  if (!file) {
    return;
  }

  try {
    await uploadAndLoad(file, true);
  } catch (error) {
    summaryText.textContent = `Upload failed: ${error.message}`;
  } finally {
    uploadAppendInput.value = "";
  }
});

clearBtn.addEventListener("click", () => {
  clearAllModels();
  summaryText.textContent = "Scene cleared.";
});
