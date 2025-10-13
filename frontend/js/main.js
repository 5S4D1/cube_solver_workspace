// ========================================
// 🚨 ANTI-REFRESH PROTECTION
// ========================================
(function() {
    'use strict';
    
    // Block ALL form submissions
    document.addEventListener('submit', function(e) {
        console.error('🚫 BLOCKED FORM SUBMIT');
        e.preventDefault();
        e.stopImmediatePropagation();
        return false;
    }, true);
    
    // Block enter key in inputs
    document.addEventListener('keydown', function(e) {
        if (e.key === 'Enter' && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT')) {
            console.warn('🚫 BLOCKED ENTER KEY');
            e.preventDefault();
            return false;
        }
    }, true);
    
    console.log('✅ Anti-refresh protection active');
})();

// ========================================
// GLOBAL VARIABLES
// ========================================
let backendURL = "";
let stream = null;
let cubeColors = {};

const colorMap = {
    U: "#ffffff", 
    R: "#ff6b00", 
    F: "#00cc44", 
    D: "#ffff00", 
    L: "#ff0000", 
    B: "#0066ff"
};

// ========================================
// INITIALIZATION
// ========================================
document.addEventListener('DOMContentLoaded', function() {
    console.log('🚀 Page loaded - Initializing...');
    
    // Load saved backend IP
    const savedIP = localStorage.getItem("backendURL");
    if (savedIP) {
        backendURL = savedIP;
        document.getElementById("backendIP").value = savedIP.replace(/^https?:\/\//, "");
        updateStatus("backendStatus", "✅ Connected: " + backendURL, "green");
    }
    
    // Initialize color grid
    initializeColorGrid();
    
    // Attach all event listeners
    setupEventListeners();
    
    console.log('✅ Initialization complete');
});

// ========================================
// EVENT LISTENERS SETUP
// ========================================
function setupEventListeners() {
    const btn = (id, handler) => {
        const element = document.getElementById(id);
        if (element) {
            element.onclick = handler;
            console.log(`✅ Attached handler to: ${id}`);
        } else {
            console.error(`❌ Element not found: ${id}`);
        }
    };
    
    btn('setBackendBtn', handleSetBackend);
    btn('uploadBtn', handleUpload);
    btn('startCameraBtn', handleStartCamera);
    btn('captureBtn', handleCapture);
    btn('stopCameraBtn', handleStopCamera);
    btn('solveBtn', handleSolve);
    btn('loadFaceBtn', handleLoadFace);
    btn('applyCorrectionBtn', handleApplyCorrection);
}

// ========================================
// UTILITY FUNCTIONS
// ========================================
function updateStatus(elementId, text, color) {
    const el = document.getElementById(elementId);
    if (el) {
        el.innerText = text;
        el.style.color = color || "black";
    }
}

function getElement(id) {
    return document.getElementById(id);
}

// ========================================
// BACKEND CONNECTION
// ========================================
async function handleSetBackend() {
    console.log('🔧 Setting backend...');
    
    let ip = getElement("backendIP").value.trim();
    if (!ip) {
        alert("Please enter backend IP!");
        return;
    }
    
    if (!ip.startsWith("http")) {
        ip = "http://" + ip;
    }
    
    updateStatus("backendStatus", "Testing connection...", "orange");
    
    try {
        const res = await fetch(`${ip}/`, { method: "GET" });
        
        if (res.ok) {
            backendURL = ip;
            localStorage.setItem("backendURL", ip);
            updateStatus("backendStatus", "✅ Connected: " + ip, "green");
            console.log('✅ Backend connected');
        } else {
            throw new Error(`HTTP ${res.status}`);
        }
    } catch (err) {
        updateStatus("backendStatus", "❌ Connection failed", "red");
        alert("Cannot connect: " + err.message);
        console.error('❌ Connection failed:', err);
    }
}

// ========================================
// FILE UPLOAD
// ========================================
async function handleUpload() {
    console.log('📤 Upload button clicked');
    
    if (!backendURL) {
        alert("Set backend IP first!");
        return;
    }
    
    const face = getElement("face").value;
    const fileInput = getElement("file");
    
    if (!fileInput.files.length) {
        alert("Select an image first!");
        return;
    }
    
    const formData = new FormData();
    formData.append("file", fileInput.files[0]);
    formData.append("face", face);
    
    updateStatus("uploadStatus", `Uploading ${face}...`, "orange");
    
    try {
        const res = await fetch(`${backendURL}/upload_face`, {
            method: "POST",
            body: formData
        });
        
        const data = await res.json();
        updateStatus("uploadStatus", `✅ ${face} uploaded successfully`, "green");
        console.log('✅ Upload successful:', data);
        
        fileInput.value = "";
        setTimeout(loadDetectedImages, 1500);
        
    } catch (err) {
        updateStatus("uploadStatus", `❌ Upload failed`, "red");
        console.error('❌ Upload error:', err);
    }
}

// ========================================
// LOAD DETECTED IMAGES
// ========================================
function loadDetectedImages() {
    if (!backendURL) return;
    
    console.log('🖼️ Loading detected images...');
    
    const faces = ["U", "R", "F", "D", "L", "B"];
    const container = getElement("facesPreview");
    container.innerHTML = "<h2>Detected Faces</h2>";
    
    faces.forEach(f => {
        const img = document.createElement("img");
        img.src = `${backendURL}/uploads/${f}_detected.jpg?t=${Date.now()}`;
        img.alt = f;
        img.style.cssText = "width:150px;margin:5px;border:2px solid #333";
        img.onerror = () => img.style.display = "none";
        container.appendChild(img);
    });
}

// ========================================
// CAMERA FUNCTIONS
// ========================================
async function handleStartCamera() {
    console.log('📷 Starting camera...');
    
    if (stream) {
        console.log('Camera already running');
        return;
    }
    
    try {
        stream = await navigator.mediaDevices.getUserMedia({ 
            video: { facingMode: "environment" } 
        });
        
        const video = getElement("cameraPreview");
        video.srcObject = stream;
        
        getElement("captureBtn").disabled = false;
        getElement("startCameraBtn").disabled = true;
        getElement("startCameraBtn").innerText = "📹 Running";
        
        console.log('✅ Camera started');
    } catch (err) {
        alert("Camera failed: " + err.message);
        console.error('❌ Camera error:', err);
    }
}

async function handleCapture() {
    console.log('📸 Capturing image...');
    
    if (!backendURL) {
        alert("Set backend IP first!");
        return;
    }
    
    if (!stream) {
        alert("Start camera first!");
        return;
    }
    
    const face = getElement("liveFace").value;
    const video = getElement("cameraPreview");
    
    const canvas = document.createElement("canvas");
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    canvas.getContext("2d").drawImage(video, 0, 0);
    
    const blob = await new Promise(resolve => 
        canvas.toBlob(resolve, "image/jpeg", 0.9)
    );
    
    const formData = new FormData();
    formData.append("file", blob, `${face}.jpg`);
    formData.append("face", face);
    
    updateStatus("uploadStatus", `Capturing ${face}...`, "orange");
    
    try {
        const res = await fetch(`${backendURL}/upload_face`, {
            method: "POST",
            body: formData
        });
        
        const data = await res.json();
        updateStatus("uploadStatus", `✅ Captured ${face}`, "green");
        console.log('✅ Capture successful');
        
        setTimeout(loadDetectedImages, 1500);
        
    } catch (err) {
        updateStatus("uploadStatus", "❌ Capture failed", "red");
        console.error('❌ Capture error:', err);
    }
}

function handleStopCamera() {
    console.log('⏹️ Stopping camera...');
    
    if (stream) {
        stream.getTracks().forEach(track => track.stop());
        stream = null;
        
        getElement("cameraPreview").srcObject = null;
        getElement("startCameraBtn").disabled = false;
        getElement("startCameraBtn").innerText = "📱 Start Back Camera";
        getElement("captureBtn").disabled = true;
        
        console.log('✅ Camera stopped');
    }
}

// ========================================
// SOLVE CUBE
// ========================================
async function handleSolve() {
    console.log('🧠 Solving cube...');
    
    if (!backendURL) {
        alert("Set backend IP first!");
        return;
    }
    
    updateStatus("solutionOutput", "Solving...", "orange");
    
    try {
        const res = await fetch(`${backendURL}/solve_cube`, { method: "POST" });
        const data = await res.json();
        
        console.log('✅ Solve response:', data);
        
        loadDetectedImages();
        
        if (data.solution && data.solution !== "Invalid cube state") {
            updateStatus("solutionOutput", "🧩 Solution: " + data.solution, "green");
        } else {
            updateStatus("solutionOutput", "⚠️ Invalid cube state", "red");
        }
        
    } catch (err) {
        updateStatus("solutionOutput", "❌ Solve failed", "red");
        console.error('❌ Solve error:', err);
    }
}

// ========================================
// COLOR CORRECTION - INITIALIZE GRID
// ========================================
function initializeColorGrid() {
    console.log('🎨 Initializing color grid...');
    
    const grid = getElement("colorGrid");
    if (!grid) {
        console.error('❌ colorGrid element not found');
        return;
    }
    
    grid.innerHTML = '';
    
    for (let i = 0; i < 9; i++) {
        const cell = document.createElement("div");
        cell.className = "color-cell";
        cell.style.cssText = `
            width: 60px;
            height: 60px;
            border: 2px solid #666;
            background: #ccc;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: bold;
            font-size: 16px;
            color: #666;
            border-radius: 5px;
            cursor: not-allowed;
        `;
        cell.textContent = "?";
        cell.dataset.index = i;
        grid.appendChild(cell);
    }
    
    console.log('✅ Grid initialized with 9 cells');
}

// ========================================
// COLOR CORRECTION - LOAD FACE
// ========================================
async function handleLoadFace() {
    console.log('🔥 Load face button clicked');
    
    if (!backendURL) {
        alert("Set backend IP first!");
        return;
    }
    
    const face = getElement("faceSelect").value.toUpperCase();
    console.log('Loading face:', face);
    
    updateStatus("correctionStatus", `Loading ${face}...`, "orange");
    
    try {
        const res = await fetch(`${backendURL}/get_detected_colors?face=${face}`);
        
        if (!res.ok) {
            throw new Error(`HTTP ${res.status}`);
        }
        
        const data = await res.json();
        console.log('✅ Got colors:', data);
        
        if (!data.colors) {
            throw new Error("No colors in response");
        }
        
        cubeColors[face] = data.colors;
        updateColorGrid(face, data.colors);
        
        updateStatus("correctionStatus", `✅ Loaded ${face} - Click cells to correct`, "green");
        
    } catch (err) {
        updateStatus("correctionStatus", `❌ Load failed: ${err.message}`, "red");
        console.error('❌ Load error:', err);
    }
}

// ========================================
// COLOR CORRECTION - UPDATE GRID
// ========================================
function updateColorGrid(face, colors) {
    console.log('🔄 Updating grid for face:', face);
    
    const grid = getElement("colorGrid");
    const flat = colors.flat();
    
    console.log('Flat colors:', flat);
    
    for (let i = 0; i < 9; i++) {
        const cell = grid.children[i];
        if (!cell) continue;
        
        const colorCode = flat[i] ? flat[i].toUpperCase() : 'U';
        const row = Math.floor(i / 3);
        const col = i % 3;
        
        cell.style.background = colorMap[colorCode] || "#ccc";
        cell.style.border = "2px solid #000";
        cell.style.cursor = "pointer";
        cell.style.color = (colorCode === 'U' || colorCode === 'D') ? "#333" : "#fff";
        cell.textContent = colorCode;
        
        cell.dataset.face = face;
        cell.dataset.row = row;
        cell.dataset.col = col;
        cell.dataset.color = colorCode;
        
        // Remove old handler and add new one
        cell.onclick = function() {
            handleCellClick(face, row, col, this);
        };
    }
    
    console.log('✅ Grid updated with colors');
}

// ========================================
// COLOR CORRECTION - CELL CLICK
// ========================================
function handleCellClick(face, row, col, cell) {
    console.log('🖱️ Cell clicked:', face, row, col);
    
    const current = cell.dataset.color;
    const newColor = prompt(
        `Correct ${face}[${row},${col}]\nCurrent: ${current}\nEnter: U, R, F, D, L, or B`,
        current
    );
    
    if (!newColor || !newColor.trim()) {
        console.log('Correction cancelled');
        return;
    }
    
    const corrected = newColor.trim().toUpperCase();
    
    if (!colorMap[corrected]) {
        alert("Invalid color! Use: U, R, F, D, L, B");
        return;
    }
    
    // Update cell appearance
    cell.dataset.color = corrected;
    cell.style.background = colorMap[corrected];
    cell.textContent = corrected;
    cell.style.color = (corrected === 'U' || corrected === 'D') ? "#333" : "#fff";
    
    // Update data structure
    if (!cubeColors[face]) {
        cubeColors[face] = [
            ['U','U','U'],
            ['U','U','U'],
            ['U','U','U']
        ];
    }
    cubeColors[face][row][col] = corrected;
    
    updateStatus("correctionStatus", `✅ Corrected ${face}[${row},${col}] → ${corrected}`, "green");
    console.log('✅ Cell corrected:', cubeColors[face]);
}

// ========================================
// COLOR CORRECTION - APPLY & SOLVE
// ========================================
async function handleApplyCorrection() {
    console.log('✅ Apply corrections clicked');
    console.log('Current cube colors:', cubeColors);
    
    if (!backendURL) {
        alert("Set backend IP first!");
        return;
    }
    
    if (Object.keys(cubeColors).length === 0) {
        alert("Load at least one face first!");
        return;
    }
    
    updateStatus("correctionStatus", "Solving cube...", "orange");
    
    try {
        const res = await fetch(`${backendURL}/manual_solve`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(cubeColors)
        });
        
        const data = await res.json();
        console.log('✅ Solve response:', data);
        
        if (data.solution && data.status === "success") {
            updateStatus("correctionStatus", `🧩 Solution: ${data.solution}`, "green");
        } else {
            updateStatus("correctionStatus", `⚠️ ${data.error || "Invalid cube"}`, "red");
        }
        
    } catch (err) {
        updateStatus("correctionStatus", `❌ Solve failed: ${err.message}`, "red");
        console.error('❌ Solve error:', err);
    }
}

console.log('✅ main.js loaded successfully');