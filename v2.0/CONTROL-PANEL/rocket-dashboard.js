/**
 * Rocket Ground Control System - Dashboard Controller
 * Handles the UI, data visualization, and command interface
 */

document.addEventListener('DOMContentLoaded', () => {
    // Application state
    const appState = {
        connected: false,
        missionStartTime: null,
        flightState: 'UNKNOWN',
        lastTelemetry: {
            timestamp: 0,
            rocketState: 0,
            altitude: 0,
            verticalSpeed: 0,
            acceleration: 0,
            temperature: 0,
            pressure: 0,
            batteryVoltage: 0,
            gpsSatellites: 0,
            gpsLatitude: 0,
            gpsLongitude: 0,
            gpsAltitude: 0,
            sensorStatus: 0,
            flags: 0,
            pitch: 0,
            roll: 0,
            yaw: 0,
            rssi: 0,
            snr: 0
        },
        lastCommand: null,
        rocketTrajectory: [],
        telemetryLog: [],
        graphsPaused: false,
        commandHistory: [],
        mapInitialized: false,
        homeLocation: null,
        rocketPath: []
    };

    // DOM Elements
    const elements = {
        // Connection elements
        connectionIndicator: document.getElementById('connection-indicator'),
        connectionText: document.getElementById('connection-text'),
        portSelector: document.getElementById('port-selector'),
        connectButton: document.getElementById('connect-button'),

        // Status elements
        missionTime: document.getElementById('mission-time'),
        rocketState: document.getElementById('rocket-state'),
        altitude: document.getElementById('altitude'),
        verticalSpeed: document.getElementById('vertical-speed'),
        acceleration: document.getElementById('acceleration'),
        temperature: document.getElementById('temperature'),
        pressure: document.getElementById('pressure'),
        battery: document.getElementById('battery'),
        batteryLevel: document.getElementById('battery-level'),
        gpsStatus: document.getElementById('gps-status'),
        rssiValue: document.getElementById('rssi-value'),
        snrValue: document.getElementById('snr-value'),
        signalBars: document.getElementById('signal-bars'),
        lastPacketTime: document.getElementById('last-packet-time'),

        // Orientation elements
        pitchValue: document.getElementById('pitch-value'),
        rollValue: document.getElementById('roll-value'),
        yawValue: document.getElementById('yaw-value'),
        rocketModel: document.getElementById('rocket-model'),

        // Graph elements
        graphTimespan: document.getElementById('graph-timespan'),
        pauseGraphs: document.getElementById('pause-graphs'),
        clearGraphs: document.getElementById('clear-graphs'),
        altitudeChart: document.getElementById('altitude-chart'),
        speedAccelChart: document.getElementById('speed-accel-chart'),

        // GPS elements
        gpsLat: document.getElementById('gps-lat'),
        gpsLon: document.getElementById('gps-lon'),
        gpsAlt: document.getElementById('gps-alt'),
        gpsSats: document.getElementById('gps-sats'),
        map: document.getElementById('map'),

        // Command elements
        lastCommand: document.getElementById('last-command'),
        commandStatus: document.getElementById('command-status'),
        cmdPing: document.getElementById('cmd-ping'),
        // cmdStatus: document.getElementById('cmd-status'),
        cmdDiagnostics: document.getElementById('cmd-diagnostics'),
        cmdCalibrate: document.getElementById('cmd-calibrate'),
        cmdWake: document.getElementById('cmd-wake'),
        cmdArm: document.getElementById('cmd-arm'),
        cmdLaunch: document.getElementById('cmd-launch'),
        cmdAbort: document.getElementById('cmd-abort'),
        cmdDeployParachute: document.getElementById('cmd-deploy-parachute'),

        // Console elements
        consoleLog: document.getElementById('console-log'),
        logLevel: document.getElementById('log-level'),
        clearConsole: document.getElementById('clear-console'),
        exportLogs: document.getElementById('export-logs'),

        // Modal elements
        confirmationModal: document.getElementById('confirmation-modal'),
        modalTitle: document.getElementById('modal-title'),
        modalMessage: document.getElementById('modal-message'),
        modalCancel: document.getElementById('modal-cancel'),
        modalConfirm: document.getElementById('modal-confirm')
    };

    // Charts configuration
    const charts = {
        altitude: null,
        speedAccel: null
    };

    // 3D Model
    let rocketModel = {
        scene: null,
        camera: null,
        renderer: null,
        rocket: null,
        animate: null,
        map: null,
        rocketMarker: null,
        homeMarker: null,
        pathLine: null
    };

    // ---- INITIALIZATION FUNCTIONS ----

    function init() {
        // Initialize charts
        initCharts();

        // Initialize 3D model
        initRocketModel();

        // Initialize map
        initMap();

        // Setup event listeners
        setupEventListeners();

        // Setup mock data for development if needed
        if (location.search.includes('demo=true')) {
            setupDemoData();
        }

        // Initialize serial bridge
        initSerialBridge();

        // Add signal strength bars
        initSignalStrengthIndicator();

        // Log initial message
        logToConsole('Sistema iniciado', 'info');
    }

    function initCharts() {
        // Configure altitude chart
        const altitudeCtx = elements.altitudeChart.getContext('2d');
        charts.altitude = new Chart(altitudeCtx, {
            type: 'line',
            data: {
                datasets: [{
                    label: 'Altitude (m)',
                    data: [],
                    borderColor: '#00aaff',
                    backgroundColor: 'rgba(0, 170, 255, 0.1)',
                    borderWidth: 2,
                    pointRadius: 0,
                    fill: true,
                    tension: 0.3
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    x: {
                        type: 'time',
                        time: {
                            unit: 'second',
                            tooltipFormat: 'HH:mm:ss'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: 'rgba(255, 255, 255, 0.6)'
                        }
                    },
                    y: {
                        beginAtZero: true,
                        title: {
                            display: true,
                            text: 'Altitude (m)',
                            color: 'rgba(255, 255, 255, 0.6)'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: 'rgba(255, 255, 255, 0.6)'
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        labels: {
                            color: 'rgba(255, 255, 255, 0.7)'
                        }
                    }
                },
                animation: false
            }
        });

        // Configure speed/acceleration chart
        const speedAccelCtx = elements.speedAccelChart.getContext('2d');
        charts.speedAccel = new Chart(speedAccelCtx, {
            type: 'line',
            data: {
                datasets: [
                    {
                        label: 'Speed (m/s)',
                        data: [],
                        borderColor: '#4caf50',
                        backgroundColor: 'rgba(76, 175, 80, 0.1)',
                        borderWidth: 2,
                        pointRadius: 0,
                        yAxisID: 'y',
                        tension: 0.3
                    },
                    {
                        label: 'Acceleration (m/s²)',
                        data: [],
                        borderColor: '#ff9800',
                        backgroundColor: 'rgba(255, 152, 0, 0.1)',
                        borderWidth: 2,
                        pointRadius: 0,
                        yAxisID: 'y1',
                        tension: 0.3
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    x: {
                        type: 'time',
                        time: {
                            unit: 'second',
                            tooltipFormat: 'HH:mm:ss'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: 'rgba(255, 255, 255, 0.6)'
                        }
                    },
                    y: {
                        title: {
                            display: true,
                            text: 'Speed (m/s)',
                            color: 'rgba(255, 255, 255, 0.6)'
                        },
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: 'rgba(255, 255, 255, 0.6)'
                        }
                    },
                    y1: {
                        position: 'right',
                        title: {
                            display: true,
                            text: 'Acceleration (m/s²)',
                            color: 'rgba(255, 255, 255, 0.6)'
                        },
                        grid: {
                            drawOnChartArea: false
                        },
                        ticks: {
                            color: 'rgba(255, 255, 255, 0.6)'
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        labels: {
                            color: 'rgba(255, 255, 255, 0.7)'
                        }
                    }
                },
                animation: false
            }
        });

        // Sync graph controls
        const secondGraphControls = document.querySelector('.graph-controls.second-graph');
        if (secondGraphControls) {
            // Clone controls from first graph to second graph
            secondGraphControls.innerHTML = document.querySelector('.graph-controls:not(.second-graph)').innerHTML;

            // Remove the ID from the cloned elements to avoid duplicate IDs
            const clonedSelect = secondGraphControls.querySelector('select');
            const clonedPauseButton = secondGraphControls.querySelector('button:first-of-type');
            const clonedClearButton = secondGraphControls.querySelector('button:last-of-type');

            if (clonedSelect) {
                clonedSelect.id = 'graph-timespan-2';
                clonedSelect.addEventListener('change', function() {
                    // Sync with main graph timespan
                    elements.graphTimespan.value = this.value;
                    updateGraphTimespan();
                });
            }

            if (clonedPauseButton) {
                clonedPauseButton.id = 'pause-graphs-2';
                clonedPauseButton.addEventListener('click', toggleGraphPause);
            }

            if (clonedClearButton) {
                clonedClearButton.id = 'clear-graphs-2';
                clonedClearButton.addEventListener('click', clearGraphs);
            }
        }
    }

    function initRocketModel() {
        // Only initialize if Three.js is available
        if (typeof THREE === 'undefined') {
            logToConsole('Three.js no disponible - El modelo 3D no será renderizado', 'warning');
            return;
        }

        // Setup scene
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x21293c);

        // Setup camera
        const camera = new THREE.PerspectiveCamera(75, elements.rocketModel.clientWidth / elements.rocketModel.clientHeight, 0.1, 1000);
        camera.position.z = 5;

        // Setup renderer
        const renderer = new THREE.WebGLRenderer({ canvas: elements.rocketModel, antialias: true });
        renderer.setSize(elements.rocketModel.clientWidth, elements.rocketModel.clientHeight);

        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
        scene.add(ambientLight);

        // Add directional light
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.6);
        directionalLight.position.set(1, 1, 1);
        scene.add(directionalLight);

        // Create a simple rocket
        const rocketGroup = new THREE.Group();

        // Rocket body
        const bodyGeometry = new THREE.CylinderGeometry(0.5, 0.5, 4, 32);
        const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0xf5f5f5 });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.position.y = 0;
        rocketGroup.add(body);

        // Rocket nose cone
        const noseGeometry = new THREE.ConeGeometry(0.5, 1.5, 32);
        const noseMaterial = new THREE.MeshPhongMaterial({ color: 0xff3d00 });
        const nose = new THREE.Mesh(noseGeometry, noseMaterial);
        nose.position.y = 2.75;
        rocketGroup.add(nose);

        // Rocket fins (4)
        const finGeometry = new THREE.BoxGeometry(0.1, 1, 1);
        const finMaterial = new THREE.MeshPhongMaterial({ color: 0x0084ff });

        for (let i = 0; i < 4; i++) {
            const fin = new THREE.Mesh(finGeometry, finMaterial);
            fin.position.y = -1.5;
            fin.position.x = Math.sin(Math.PI * i / 2) * 0.55;
            fin.position.z = Math.cos(Math.PI * i / 2) * 0.55;
            fin.rotation.y = Math.PI * i / 2;
            rocketGroup.add(fin);
        }

        // Add rocket to scene
        scene.add(rocketGroup);

        // Add reference axes
        const axesHelper = new THREE.AxesHelper(3);
        scene.add(axesHelper);

        // Store in the rocket model object
        rocketModel.scene = scene;
        rocketModel.camera = camera;
        rocketModel.renderer = renderer;
        rocketModel.rocket = rocketGroup;

        // Animation function
        function animate() {
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        }

        // Store and start animation
        rocketModel.animate = animate;
        animate();

        // Handle window resize
        window.addEventListener('resize', () => {
            if (rocketModel.camera && rocketModel.renderer) {
                rocketModel.camera.aspect = elements.rocketModel.clientWidth / elements.rocketModel.clientHeight;
                rocketModel.camera.updateProjectionMatrix();
                rocketModel.renderer.setSize(elements.rocketModel.clientWidth, elements.rocketModel.clientHeight);
            }
        });
    }

    function initMap() {
        // Check if Leaflet is available
        if (typeof L === 'undefined') {
            logToConsole('Leaflet library not available - Map will not be rendered', 'warning');
            return;
        }

        try {
            // Initialize Leaflet map
            const map = L.map('map', {
                attributionControl: false,
                zoomControl: true,
                dragging: true,
                doubleClickZoom: true
            }).setView([40.4168, -3.7038], 13); // Default view

            // Add dark mode tile layer
            L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>',
                subdomains: 'abcd',
                maxZoom: 21
            }).addTo(map);

            // Create custom rocket icon
            const rocketIcon = L.divIcon({
                className: 'rocket-marker',
                html: '<i class="fas fa-rocket"></i>',
                iconSize: [20, 20]
            });

            // Create custom home/launch site icon
            const homeIcon = L.divIcon({
                className: 'home-marker',
                html: '<i class="fas fa-home"></i>',
                iconSize: [20, 20]
            });

            // Initialize markers (not yet added to map)
            const rocketMarker = L.marker([0, 0], { icon: rocketIcon });
            const homeMarker = L.marker([0, 0], { icon: homeIcon });

            // Initialize path line (empty)
            const pathLine = L.polyline([], {
                className: 'rocket-path',
                color: '#ff3d00',
                weight: 3,
                opacity: 0.7
            });

            // Store map objects for later use
            appState.mapInitialized = true;
            rocketModel.map = map;
            rocketModel.rocketMarker = rocketMarker;
            rocketModel.homeMarker = homeMarker;
            rocketModel.pathLine = pathLine;

            // Log success
            logToConsole('Map initialized successfully', 'info');
        } catch (error) {
            logToConsole(`Error initializing map: ${error.message}`, 'error');
        }
    }

    function initSignalStrengthIndicator() {
        // Create 5 signal bars
        const signalBars = elements.signalBars;

        for (let i = 0; i < 5; i++) {
            const bar = document.createElement('div');
            bar.className = 'signal-bar inactive';
            signalBars.appendChild(bar);
        }
    }

    // ---- EVENT LISTENERS ----

    // ---- EVENT LISTENERS ACTUALIZADOS ----

    function setupEventListeners() {
        // Connection button
        elements.connectButton.addEventListener('click', toggleConnection);

        // Command buttons - Actualizados sin despertar y armar
        elements.cmdPing.addEventListener('click', () => sendCommand('PING'));
        // elements.cmdStatus.addEventListener('click', () => sendCommand('GET_STATUS'));
        elements.cmdDiagnostics.addEventListener('click', () => sendCommand('RUN_DIAGNOSTICS'));
        elements.cmdCalibrate.addEventListener('click', () => sendCommand('CALIBRATE_SENSORS'));

        // Critical commands with confirmation
        elements.cmdLaunch.addEventListener('click', () => showConfirmation('INICIAR LANZAMIENTO', '¡ADVERTENCIA! Estás a punto de iniciar la secuencia de lanzamiento. ¿Estás COMPLETAMENTE seguro?', () => sendCommand('LAUNCH_COMMAND')));
        elements.cmdAbort.addEventListener('click', () => showConfirmation('ABORTAR MISIÓN', '¡ADVERTENCIA! Estás a punto de abortar la misión. ¿Estás COMPLETAMENTE seguro?', () => sendCommand('ABORT_COMMAND')));
        elements.cmdDeployParachute.addEventListener('click', () => showConfirmation('DESPLIEGUE DE EMERGENCIA', '¡ADVERTENCIA! Estás a punto de desplegar el paracaídas manualmente. ¿Estás COMPLETAMENTE seguro?', () => sendCommand('FORCE_DEPLOY_PARACHUTE')));

        // Graph controls
        elements.graphTimespan.addEventListener('change', updateGraphTimespan);
        elements.pauseGraphs.addEventListener('click', toggleGraphPause);
        elements.clearGraphs.addEventListener('click', clearGraphs);

        // Console controls
        elements.logLevel.addEventListener('change', filterLogs);
        elements.clearConsole.addEventListener('click', clearConsole);
        elements.exportLogs.addEventListener('click', exportLogs);

        // Modal controls
        elements.modalCancel.addEventListener('click', hideConfirmation);

        // Window resize event for responsive design
        window.addEventListener('resize', handleResize);
    }

    // ---- CONNECTION HANDLING ----

    function initSerialBridge() {
        // Check if the Serial Bridge API is available
        if (typeof SerialBridge === 'undefined') {
            logToConsole('SerialBridge no disponible - Trabajando en modo simulación', 'warning');

            // Add simulated devices for testing
            populatePortSelector(['COM1', 'COM2', 'COM3', '/dev/ttyUSB0']);
            return;
        }

        // Initialize the Serial Bridge
        SerialBridge.init({
            onPortsDiscovered: populatePortSelector,
            onConnect: handleConnect,
            onDisconnect: handleDisconnect,
            onData: handleIncomingData,
            onError: handleSerialError
        });

        // Request available ports
        SerialBridge.listPorts();
    }

    function populatePortSelector(ports) {
        // Clear existing options (keep the first one)
        while (elements.portSelector.options.length > 1) {
            elements.portSelector.remove(1);
        }

        // Add new options
        ports.forEach(port => {
            const option = document.createElement('option');
            option.value = port;
            option.textContent = port;
            elements.portSelector.appendChild(option);
        });
    }

    function toggleConnection() {
        if (appState.connected) {
            disconnectFromRocket();
        } else {
            connectToRocket();
        }
    }

    function connectToRocket() {
        const selectedPort = elements.portSelector.value;

        if (!selectedPort) {
            logToConsole('Por favor, selecciona un puerto', 'error');
            return;
        }

        // Update UI to connecting state
        updateConnectionStatus('connecting');

        // If we're in demo mode or SerialBridge is not available, simulate connection
        if (location.search.includes('demo=true') || typeof SerialBridge === 'undefined') {
            setTimeout(() => {
                handleConnect();
                setupDemoData();
            }, 1000);
            return;
        }

        // Try to connect using SerialBridge
        SerialBridge.connect(selectedPort);
    }

    function disconnectFromRocket() {
        // If we're in demo mode, just simulate disconnect
        if (location.search.includes('demo=true') || typeof SerialBridge === 'undefined') {
            handleDisconnect();
            return;
        }

        // Disconnect using SerialBridge
        SerialBridge.disconnect();
    }

    function handleConnect() {
        appState.connected = true;
        updateConnectionStatus('online');
        logToConsole('Conectado al cohete', 'info');
        elements.connectButton.textContent = 'Desconectar';
        //
        // // Send an initial status request
        // setTimeout(() => sendCommand('GET_STATUS'), 500);
    }

    function handleDisconnect() {
        appState.connected = false;
        updateConnectionStatus('offline');
        logToConsole('Desconectado del cohete', 'info');
        elements.connectButton.textContent = 'Conectar';
    }

    function updateConnectionStatus(status) {
        elements.connectionIndicator.className = `indicator ${status}`;

        switch (status) {
            case 'offline':
                elements.connectionText.textContent = 'Desconectado';
                break;
            case 'connecting':
                elements.connectionText.textContent = 'Connecting...';
                break;
            case 'online':
                elements.connectionText.textContent = 'Conectado';
                break;
        }
    }

    function handleSerialError(error) {
        logToConsole(`Error de comunicación: ${error}`, 'error');
        handleDisconnect();
    }

    // ---- DATA HANDLING ----

    function handleIncomingData(data) {
        try {
            // Try to parse as JSON
            const packet = JSON.parse(data);

            // Process by packet type
            switch (packet.type) {
                case 'telemetry':
                    processTelemetryData(packet.data);
                    break;
                case 'status':
                    processStatusData(packet.data);
                    break;
                case 'command_response':
                    processCommandResponse(packet.data);
                    break;
                case 'diagnostic':
                    processDiagnosticData(packet.data);
                    break;
                case 'event':
                    processEventNotification(packet.data);
                    break;
                default:
                    // logToConsole(`Paquete desconocido: ${data} asumiendo que es telemtria`, 'warning');
                    processTelemetryData(packet);
            }
        } catch (error) {
            // Not JSON, handle as raw data
            logToConsole(`Datos recibidos: ${data}`, 'info');
        }
    }

    function processTelemetryData(telemetry) {
        // Store in app state
        appState.lastTelemetry = {
            ...appState.lastTelemetry,
            ...telemetry,
            receivedAt: Date.now()
        };

        // If this is the first telemetry after connection, set mission start time
        if (!appState.missionStartTime) {
            appState.missionStartTime = Date.now() - (telemetry.ts || 0);
        }

        // Update UI
        updateTelemetryDisplay(telemetry);
        updateOrientationDisplay(telemetry);

        // Add to charts if not paused
        if (!appState.graphsPaused) {
            addDataToCharts(telemetry);
        }

        // Update GPS if available
        if (telemetry.gpsS > 0) {
            updateGPSDisplay(telemetry);
        }

        // Add to telemetry log
        appState.telemetryLog.push({
            timestamp: Date.now(),
            data: { ...telemetry }
        });

        // Log the telemetry packet
        logToConsole(`Telemetría recibida [Alt: ${telemetry.alt.toFixed(1)}m, V: ${telemetry.vS.toFixed(1)}m/s]`, 'telemetry');
    }

    function processStatusData(status) {
        // Update state display
        updateRocketStateDisplay(status.rocketState);

        // Update mission time if needed
        if (status.missionTime && !appState.missionStartTime) {
            appState.missionStartTime = Date.now() - status.missionTime;
        }

        // Update other status displays
        updateBatteryDisplay(status.batteryVoltage, status.batteryPercentage);
        updateSignalDisplay(status.rssi, status.snr);

        // Log the status
        logToConsole(`Estado recibido: ${getRocketStateName(status.rocketState)}`, 'info');
    }

    function processCommandResponse(response) {
        // Update command status
        elements.commandStatus.textContent = response.success ? 'Éxito' : 'Fallido';
        elements.commandStatus.style.color = response.success ? '#4caf50' : '#f44336';

        // Log the response
        const message = response.message || (response.success ? 'Comando ejecutado con éxito' : 'Error ejecutando comando');
        logToConsole(`Respuesta: ${message}`, response.success ? 'info' : 'error');

        // If it was a ping, calculate round-trip time
        if (appState.lastCommand && appState.lastCommand.command === 'PING' && response.success) {
            const rtt = Date.now() - appState.lastCommand.timestamp;
            logToConsole(`Ping RTT: ${rtt}ms`, 'info');
        }
    }

    function processDiagnosticData(diagnostic) {
        // Create a formatted diagnostic log
        let message = 'Resultados de Diagnóstico:\n';

        diagnostic.tests.forEach((test, index) => {
            message += `${index + 1}. ${test.name}: ${test.passed ? 'PASÓ' : 'FALLÓ'}\n`;
            if (!test.passed && test.error) {
                message += `   Error: ${test.error}\n`;
            }
        });

        // Log to console
        logToConsole(message, 'info');
    }

    function processEventNotification(event) {
        // Handle event notification
        logToConsole(`EVENTO: ${event.description}`, 'warning');

        // Play alert sound if it's critical
        if (event.priority === 'critical' || event.priority === 'emergency') {
            playAlertSound();
        }
    }

    // ---- UI UPDATES ----

    function updateTelemetryDisplay(telemetry) {
        // console.log(telemetry);

        // Update numeric values
        elements.altitude.textContent = `${telemetry.alt.toFixed(1)} m`;
        elements.verticalSpeed.textContent = `${telemetry.vS.toFixed(1)} m/s`;
        elements.acceleration.textContent = `${telemetry.acc.toFixed(2)} m/s²`;
        elements.temperature.textContent = `${telemetry.tem.toFixed(1)} °C`;
        elements.pressure.textContent = `${telemetry.pres.toFixed(1)} hPa`;

        // Update last packet time
        const now = new Date();
        elements.lastPacketTime.textContent = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;

        // Update mission time
        updateMissionTime();

        // Update state if included
        if (telemetry.state !== undefined) {
            updateRocketStateDisplay(telemetry.state);
        }

        // Update battery if included
        if (telemetry.bV !== undefined) {
            updateBatteryDisplay(telemetry.bV);
        }

        // Update signal if included
        if (telemetry.rssi !== undefined && telemetry.snr !== undefined) {
            updateSignalDisplay(telemetry.rssi, telemetry.snr);
        }
    }

    function updateOrientationDisplay(telemetry) {
        // Update numeric orientation values
        if (telemetry.pitch !== undefined) {
            elements.pitchValue.textContent = `${telemetry.pitch.toFixed(1)}°`;
        }

        if (telemetry.roll !== undefined) {
            elements.rollValue.textContent = `${telemetry.roll.toFixed(1)}°`;
        }

        if (telemetry.yaw !== undefined) {
            elements.yawValue.textContent = `${telemetry.yaw.toFixed(1)}°`;
        }

        // Update 3D model orientation
        if (rocketModel.rocket && telemetry.pitch !== undefined && telemetry.roll !== undefined && telemetry.yaw !== undefined) {
            // Convert to radians
            const pitch = telemetry.pitch * Math.PI / 180;
            const roll = telemetry.roll * Math.PI / 180;
            const yaw = telemetry.yaw * Math.PI / 180;

            // Reset rotation
            rocketModel.rocket.rotation.set(0, 0, 0);

            // Apply rotations in correct order (yaw, pitch, roll)
            rocketModel.rocket.rotateY(yaw);    // Yaw around Y-axis
            rocketModel.rocket.rotateX(pitch);  // Pitch around X-axis
            rocketModel.rocket.rotateZ(roll);   // Roll around Z-axis
        }
    }

    function updateRocketStateDisplay(stateCode) {
        //Parse the String to an integer
        stateCode = parseInt(stateCode);

        const stateName = getRocketStateName(stateCode);
        const stateClass = getRocketStateClass(stateCode);

        // Update displayed state
        elements.rocketState.textContent = stateName;

        // Remove all state classes and add current one
        elements.rocketState.className = 'status-value highlight ' + stateClass;

        // Store current flight state
        appState.flightState = stateName;
    }

    function updateBatteryDisplay(voltage, percentage) {
        // Update voltage text
        elements.battery.textContent = `${voltage.toFixed(2)} V`;

        // Calculate percentage if not provided
        if (percentage === undefined) {
            // Assuming 3.0V is empty and 4.2V is full for LiPo
            percentage = Math.min(100, Math.max(0, ((voltage - 3.0) / 1.2) * 100));
        }

        // Update battery level indicator
        elements.batteryLevel.style.width = `${percentage}%`;

        // Add appropriate class based on level
        elements.batteryLevel.className = '';
        if (percentage < 20) {
            elements.batteryLevel.classList.add('critical');
        } else if (percentage < 50) {
            elements.batteryLevel.classList.add('warning');
        }
    }

    function updateSignalDisplay(rssi, snr) {
        // Update RSSI and SNR text
        elements.rssiValue.textContent = `${rssi} dBm`;
        elements.snrValue.textContent = `${snr.toFixed(1)} dB`;

        // Update signal bars
        // RSSI typically ranges from -30 (excellent) to -120 (terrible)
        const signalBars = elements.signalBars.querySelectorAll('.signal-bar');
        const signalStrength = Math.min(5, Math.max(0, Math.floor((rssi + 120) / 20)));

        signalBars.forEach((bar, index) => {
            if (index < signalStrength) {
                bar.classList.remove('inactive');
            } else {
                bar.classList.add('inactive');
            }
        });
    }

    function updateGPSDisplay(telemetry) {
        // Update GPS coordinates text
        elements.gpsLat.textContent = `${telemetry.gpsLat.toFixed(6)}°`;
        elements.gpsLon.textContent = `${telemetry.gpsLong.toFixed(6)}°`;
        elements.gpsAlt.textContent = `${telemetry.gpsAlt.toFixed(1)} m`;
        elements.gpsSats.textContent = telemetry.gpsS;

        // Update GPS status text
        elements.gpsStatus.textContent = telemetry.gpsS > 0
            ? `Fix (${telemetry.gpsS} satélites)`
            : 'Sin Señal';

        // If Leaflet map is initialized and we have valid coordinates, update the map
        if (appState.mapInitialized &&
            telemetry.gpsLat !== 0 &&
            telemetry.gpsLong !== 0 &&
            telemetry.gpsS > 0) {

            const position = [telemetry.gpsLat, telemetry.gpsLong];

            // Set home position if not set yet
            if (!appState.homeLocation && telemetry.gpsAlt < 10) {
                appState.homeLocation = position;
                rocketModel.homeMarker.setLatLng(position).addTo(rocketModel.map);
                logToConsole(`Home location set: ${position[0].toFixed(6)}, ${position[1].toFixed(6)}`, 'info');
            }

            // Update rocket marker position
            rocketModel.rocketMarker.setLatLng(position);
            if (!rocketModel.map.hasLayer(rocketModel.rocketMarker)) {
                rocketModel.rocketMarker.addTo(rocketModel.map);
            }

            // Add point to path and update line
            appState.rocketPath.push(position);
            rocketModel.pathLine.setLatLngs(appState.rocketPath);
            if (!rocketModel.map.hasLayer(rocketModel.pathLine)) {
                rocketModel.pathLine.addTo(rocketModel.map);
            }

            // Center map on rocket if following is enabled
            // (You could add a "follow" button to toggle this behavior)
            rocketModel.map.setView(position);

            // Update tooltip with altitude information
            rocketModel.rocketMarker.bindTooltip(`Altitude: ${telemetry.alt.toFixed(1)}m<br>Speed: ${telemetry.vS.toFixed(1)}m/s`).openTooltip();
        }
    }

    function clearMapTrajectory() {
        if (appState.mapInitialized) {
            appState.rocketPath = [];
            if (rocketModel.pathLine) {
                rocketModel.pathLine.setLatLngs([]);
            }
            logToConsole('Map trajectory cleared', 'info');
        }
    }

    function updateMissionTime() {
        if (!appState.missionStartTime) return;

        const elapsed = Date.now() - appState.missionStartTime;
        const hours = Math.floor(elapsed / 3600000).toString().padStart(2, '0');
        const minutes = Math.floor((elapsed % 3600000) / 60000).toString().padStart(2, '0');
        const seconds = Math.floor((elapsed % 60000) / 1000).toString().padStart(2, '0');

        elements.missionTime.textContent = `${hours}:${minutes}:${seconds}`;
    }

    // ---- CHART FUNCTIONS ----

    function addDataToCharts(telemetry) {
        const timestamp = telemetry.ts ? new Date(appState.missionStartTime + telemetry.ts) : new Date();

        // Add data to altitude chart
        if (charts.altitude) {
            charts.altitude.data.datasets[0].data.push({
                x: timestamp,
                y: telemetry.alt
            });

            // Limit data points to avoid performance issues
            if (charts.altitude.data.datasets[0].data.length > 1000) {
                charts.altitude.data.datasets[0].data.shift();
            }

            charts.altitude.update();
        }

        // Add data to speed/acceleration chart
        if (charts.speedAccel) {
            // Add vertical speed
            charts.speedAccel.data.datasets[0].data.push({
                x: timestamp,
                y: telemetry.vS
            });

            // Add acceleration
            charts.speedAccel.data.datasets[1].data.push({
                x: timestamp,
                y: telemetry.acc
            });

            // Limit data points
            if (charts.speedAccel.data.datasets[0].data.length > 1000) {
                charts.speedAccel.data.datasets[0].data.shift();
                charts.speedAccel.data.datasets[1].data.shift();
            }

            charts.speedAccel.update();
        }
    }

    function updateGraphTimespan() {
        const timespan = parseInt(elements.graphTimespan.value, 10) * 1000; // Convert to milliseconds
        const now = Date.now();

        // Update altitude chart
        if (charts.altitude) {
            charts.altitude.options.scales.x.min = new Date(now - timespan);
            charts.altitude.options.scales.x.max = new Date(now);
            charts.altitude.update();
        }

        // Update speed/acceleration chart
        if (charts.speedAccel) {
            charts.speedAccel.options.scales.x.min = new Date(now - timespan);
            charts.speedAccel.options.scales.x.max = new Date(now);
            charts.speedAccel.update();
        }
    }

    function toggleGraphPause() {
        appState.graphsPaused = !appState.graphsPaused;

        // Update button icon
        elements.pauseGraphs.innerHTML = appState.graphsPaused
            ? '<i class="fas fa-play"></i>'
            : '<i class="fas fa-pause"></i>';

        // Log pause state
        logToConsole(`Gráficos ${appState.graphsPaused ? 'pausados' : 'reanudados'}`, 'info');
    }

    function clearGraphs() {
        // Clear altitude chart
        if (charts.altitude) {
            charts.altitude.data.datasets[0].data = [];
            charts.altitude.update();
        }

        // Clear speed/acceleration chart
        if (charts.speedAccel) {
            charts.speedAccel.data.datasets[0].data = [];
            charts.speedAccel.data.datasets[1].data = [];
            charts.speedAccel.update();
        }

        logToConsole('Gráficos limpiados', 'info');
    }

    // ---- COMMAND FUNCTIONS ----

    function sendCommand(command, params = {}) {
        // Store command in history
        appState.lastCommand = {
            command,
            params,
            timestamp: Date.now()
        };

        // Update UI
        elements.lastCommand.textContent = command;
        elements.commandStatus.textContent = 'Enviando...';
        elements.commandStatus.style.color = '#ffc107';

        // Log the command
        logToConsole(`Enviando comando: ${command}`, 'command');

        // Add to command history
        appState.commandHistory.push({
            timestamp: Date.now(),
            command,
            params
        });

        // If we're in demo mode, simulate command response
        if (location.search.includes('demo=true') || typeof SerialBridge === 'undefined') {
            simulateCommandResponse(command);
            return;
        }

        // Send the command through SerialBridge
        SerialBridge.sendCommand({
            command,
            params
        });
    }

    function showConfirmation(title, message, confirmCallback) {
        // Set modal content
        elements.modalTitle.textContent = title;
        elements.modalMessage.textContent = message;

        // Set confirm button action
        elements.modalConfirm.onclick = () => {
            confirmCallback();
            hideConfirmation();
        };

        // Show the modal
        elements.confirmationModal.classList.add('active');
    }

    function hideConfirmation() {
        elements.confirmationModal.classList.remove('active');
    }

    // ---- CONSOLE FUNCTIONS ----

    function logToConsole(message, level = 'info') {
        const timestamp = new Date();
        const timeString = `${timestamp.getHours().toString().padStart(2, '0')}:${timestamp.getMinutes().toString().padStart(2, '0')}:${timestamp.getSeconds().toString().padStart(2, '0')}`;

        // Create log entry
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${level}`;
        logEntry.innerHTML = `<span class="log-time">${timeString}</span> ${message}`;

        // Add to console
        elements.consoleLog.appendChild(logEntry);

        // Scroll to bottom
        elements.consoleLog.scrollTop = elements.consoleLog.scrollHeight;

        // Limit log entries
        while (elements.consoleLog.children.length > 1000) {
            elements.consoleLog.removeChild(elements.consoleLog.firstChild);
        }
    }

    function filterLogs() {
        const level = elements.logLevel.value;
        const logEntries = elements.consoleLog.querySelectorAll('.log-entry');

        logEntries.forEach(entry => {
            if (level === 'all' || entry.classList.contains(`log-${level}`)) {
                entry.style.display = 'block';
            } else {
                entry.style.display = 'none';
            }
        });
    }

    function clearConsole() {
        elements.consoleLog.innerHTML = '';
        logToConsole('Consola limpiada', 'info');
    }

    function exportLogs() {
        // Create CSV content
        let csv = 'Timestamp,Level,Message\n';

        appState.telemetryLog.forEach(log => {
            const timestamp = new Date(log.timestamp).toISOString();
            const telemetry = log.data;

            csv += `${timestamp},telemetry,State: ${getRocketStateName(telemetry.state)},Altitude: ${telemetry.alt.toFixed(1)}m, VerticalSpeed: ${telemetry.vS.toFixed(1)}m/s, Acceleration: ${telemetry.acc.toFixed(2)}m/s²,Temperature: ${telemetry.tem.toFixed(1)}°C,Pressure: ${telemetry.pres.toFixed(1)}hPa,BatteryVoltage: ${telemetry.bV.toFixed(2)}V,GPSLat: ${telemetry.gpsLat.toFixed(6)}°,GPSLong: ${telemetry.gpsLong.toFixed(6)}°,GPSAlt: ${telemetry.gpsAlt.toFixed(1)}m,GPSSat: ${telemetry.gpsS},RSSI: ${telemetry.rssi}dBm,SNR: ${telemetry.snr.toFixed(1)}dB",Flags: ${telemetry.flgs.join(',')}\n`;
        });

        appState.commandHistory.forEach(log => {
            const timestamp = new Date(log.timestamp).toISOString();
            csv += `${timestamp},command,"${log.command}"\n`;
        });

        // Create download link
        const blob = new Blob([csv], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');

        link.href = url;
        link.download = `rocket_log_${new Date().toISOString().replace(/:/g, '-')}.csv`;
        link.click();

        logToConsole('Logs exportados a CSV', 'info');
    }

    // ---- UTILITY FUNCTIONS ----

    function getRocketStateName(stateCode) {
        const states = {
            0x01: 'INICIALIZANDO',
            0x02: 'REPOSO',
            0x03: 'LISTO',
            0x10: 'PROPULSIÓN',
            0x11: 'ASCENSO INERCIAL',
            0x12: 'APOGEO',
            0x13: 'DESCENSO',
            0x14: 'DESCENSO CON PARACAÍDAS',
            0x20: 'ATERRIZADO',
            0xE0: 'ERROR'
        };

        return states[stateCode] || 'DESCONOCIDO';
    }

    function getRocketStateClass(stateCode) {
        const stateClasses = {
            0x01: 'state-idle',
            0x02: 'state-idle',
            0x03: 'state-ready',
            0x04: 'state-armed',
            0x05: 'state-countdown',
            0x10: 'state-flight',
            0x11: 'state-flight',
            0x12: 'state-flight',
            0x13: 'state-descent',
            0x14: 'state-descent',
            0x20: 'state-landed',
            0xE0: 'state-error'
        };

        return stateClasses[stateCode] || '';
    }

    function handleResize() {
        // Update 3D model size if it exists
        if (rocketModel.camera && rocketModel.renderer) {
            rocketModel.camera.aspect = elements.rocketModel.clientWidth / elements.rocketModel.clientHeight;
            rocketModel.camera.updateProjectionMatrix();
            rocketModel.renderer.setSize(elements.rocketModel.clientWidth, elements.rocketModel.clientHeight);
        }

        // Update charts
        if (charts.altitude) {
            charts.altitude.resize();
        }

        if (charts.speedAccel) {
            charts.speedAccel.resize();
        }

        if (appState.mapInitialized && rocketModel.map) {
            rocketModel.map.invalidateSize();
        }
    }

    function playAlertSound() {
        // Play an alert sound
        const audio = new Audio('data:audio/wav;base64,UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJermZKwe4lYFz1Zn8z///7//e2XOAAA//90FwAAAAAAAAAAAAAAIgj3XL3a//////////95HwMgAAAAAAAAAAAAsZGgnp6dgW1bALGql5OufIdYG0Bbn8r8/v/////+7Zg5AAD//3UYAGz/////ayYPlcv//6lZEQAAAAAAAAAAAP//y6EaAAAAAAAAAACxkaCfnp2BbVsAr6mUkrFoglkYQFmexfv+/////f/tl4E5AAD//3UYAG7/////ah4Om8n//6dXEQAAAAAAAAAAAP//y6CZAAAAAAAAAP//E/IBAAAAAAAAsZKgoJ6dgm9aAK6pj42ugIVaGkFZncL7/v/////+7le8QQAA//91GQBu/////2kaB53O//+tVBQAAAAAAAAAAAD//8uhmQAAAAAAAAAA//8T8gEAAAAAAABqDUuoqzFaY7S1noNqOB2kxJAA/v////7/7hHcTAAA//91GgBv/////2gXBJ/P//+uUhUAAAAAAAAAAAD//8uimQAAAAAAAAAA//8T8gEAAAAAAABmDD9jqyRKZbW2n4ZtOhuiwYwA/f////9b3m8NAP//dRsAb/////9nFQKgz///r1AWAAAAAAAAAAAAAP//zKKaAAAAAAAAAAD//xPyAQAAAAAAAGcKT1GqGTZouLehiGw3G6K9iAD9/////1ved3MI//91HABw/////2YTAaLP//+vThgAAAAAAAAAAAD//82imgAAAAAAAAAA//8T8gEAAAAAAABpDmhBqgYle7m4pIlqNhuiuYYA/f////9a3ndDB///dR0AcP////9lEQCj0P//sE0ZAAAAAAAAAAAAAP//zaOaAAAAAAAAAAD//xPyAQAAAAAAAGYOdTaq9wuIurmmiWUyG6G1gwD9/////1reekAG//91HgBy/////2QQAKbQ//+xSxsAAAAAAAAAAAD//82jmgAAAAAAAAAA//8T8gEAAAAAAABnEIMmp+f3kru7qIliLRqespEA/f////9Z3nx9BP//dR8AcP////9iDf2o0P//sUocAAAAAAAAAAAA//7OpJoAAAAAAAAAAP//E/IBAAAAAAAAaROQFafX5JS9vamJYCoboK6OAP3/////WN6AFAL//3YgAHP/////Ygz8qtD//7JIIAAAAAAAAAAAAAD//s6lmwAAAAAAAAAA//8T8gEAAAAAAABrFJwGpdPUmL2+q4lelBifqowA/f////9Y3oSSAP//diEAdP////9gCfqr0P//s0UiAAAAAAAAAAAA//7Pp5sAAAAAAAAAAP//E/IBAAAAAAAAbBenA6PPxpy+v62KW4sYnqaJAP3/////V96GFAD//3YiAHT/////YAj5rNH//7RDJQAAAAAAAAAAAAAA/v7QqJwAAAAAAAAAAP//E/IBAAAAAAAAaxmoAKLLuZ++wK6KWYIYnqKHAP3/////Vt6I2QD//3YjAHb/////Xgb3rtH//7VBJAAAAAAAAAAAAAD+/tGonAAAAAAAAAAA//8T8gEAAAAAAABsGqoAndCxo7/BsIpWfBidnoUA/f////9V3outdQD//3YkAHb/////Xgb0r9H//7Y/JgAAAAAAAAAAAP790qmdAAAAAAAAAAD//xPyAQAAAAAAAGsZrACb1Kmlv8KyilN3GJ2ahAD9/////1PejiDv//92JQB3/////10D9LHR//+2PScAAAAAAAAAAAD+/dOpnQAAAAAAAAAA//8T8gEAAAAAAABtGa0AmNinp7/EtYRDRm4tAAAAAAAAAADNhQDA//92JgB4/////1wC8rPR//+3OykAAAAAAAAAAAD+/dSqnQAAAAAAAAAA//8T8gEAAAAAAABuGa4GmUBtcGsAAAAAAAAAAAAAAAAAAAAAAAAA//92JwB5/////1wB8rTR//+3OisAAAAAAAAAAAD+/dWqngAAAAAAAAAA//8T8gEAAAAAAABwGa8P6ejo2AEAAAAAAAAAAAAAAAAAAAAAAAAAY5cAef////9bAO+20f//uDgtAAAAAAAAAAAA/vzWq54AAAAAAAAAAP//E/IBAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA==');
        audio.play();
    }

    // ---- DEMO DATA FUNCTIONS ----

    function setupDemoData() {
        // Start with a base altitude
        let altitude = 0;
        let verticalSpeed = 0;
        let acceleration = 0;
        let state = 0x02; // IDLE
        let missionTime = 0;
        let flightPhase = 'pre-launch';

        // Set mission start time
        appState.missionStartTime = Date.now();

        // Simulate telemetry at regular intervals
        setInterval(() => {
            if (!appState.connected) return;

            missionTime += 1000; // 1 second in milliseconds

            // Simulate different flight phases
            switch (flightPhase) {
                case 'pre-launch':
                    // Just stay on the ground
                    altitude = 0;
                    verticalSpeed = 0;
                    acceleration = 0;
                    break;

                case 'powered-flight':
                    // Accelerate upward
                    acceleration = 20 + Math.random() * 5; // 20-25 m/s²
                    verticalSpeed += acceleration * 1; // 1 second update interval
                    altitude += verticalSpeed * 1;

                    // Transition to coast after 12 seconds
                    if (missionTime > 12000) {
                        flightPhase = 'coast';
                        state = 0x11; // COASTING
                    }
                    break;

                case 'coast':
                    // Decelerate due to gravity
                    acceleration = -9.8 + Math.random() * 1 - 0.5; // ~9.8 m/s² (gravity)
                    verticalSpeed += acceleration * 1;
                    altitude += verticalSpeed * 1;

                    // Detect apogee (peak altitude)
                    if (verticalSpeed < 0 && Math.abs(verticalSpeed) > 5) {
                        flightPhase = 'descent';
                        state = 0x13; // DESCENT
                    }

                    // Detect apogee
                    if (verticalSpeed < 0 && Math.abs(verticalSpeed) < 5) {
                        // Briefly show APOGEE state
                        state = 0x12; // APOGEE
                        setTimeout(() => {
                            if (appState.connected) {
                                // Then move to descent with parachute
                                flightPhase = 'parachute';
                                state = 0x14; // PARACHUTE_DESCENT
                                verticalSpeed = -5; // Parachute slows descent
                            }
                        }, 2000);
                    }
                    break;

                case 'descent':
                    // Fall faster due to gravity
                    acceleration = -9.8 + Math.random() * 1 - 0.5;
                    verticalSpeed += acceleration * 0.5; // 0.5 second update interval
                    altitude += verticalSpeed * 0.5;

                    // Limit terminal velocity
                    if (verticalSpeed < -50) {
                        verticalSpeed = -50;
                    }
                    break;

                case 'parachute':
                    // Slow descent with parachute
                    acceleration = 0;
                    verticalSpeed = -5 + Math.random() * 2 - 1; // -4 to -6 m/s
                    altitude += verticalSpeed * 1;
                    break;

                case 'landed':
                    // On the ground
                    altitude = 0;
                    verticalSpeed = 0;
                    acceleration = 0;
                    state = 0x20; // LANDED
                    break;
            }

            // Don't go below ground
            if (altitude < 0) {
                altitude = 0;
                verticalSpeed = 0;
                acceleration = 0;

                if (flightPhase === 'parachute' || flightPhase === 'descent') {
                    flightPhase = 'landed';
                    state = 0x20; // LANDED
                }
            }

            // Generate telemetry packet
            const telemetry = {
                timestamp: missionTime,
                rocketState: state,
                altitude: altitude,
                verticalSpeed: verticalSpeed,
                acceleration: acceleration,
                temperature: 25 - altitude / 100, // Temperature decreases with altitude
                pressure: 1013 - altitude / 10, // Pressure decreases with altitude
                batteryVoltage: 4.2 - missionTime / 1000000, // Battery slowly drains
                gpsSatellites: 8,
                gpsLatitude: 40.7128 + (Math.random() * 0.01 - 0.005), // Random walk around New York
                gpsLongitude: -74.006 + (Math.random() * 0.01 - 0.005),
                gpsAltitude: altitude,
                pitch: Math.sin(missionTime / 10000) * 10, // Oscillate for demo
                roll: Math.sin(missionTime / 8000) * 15,
                yaw: Math.sin(missionTime / 12000) * 5,
                rssi: -60 - Math.random() * 20, // Signal strength
                snr: 12 + Math.random() * 6 - 3 // Signal-to-noise ratio
            };

            // Process the telemetry
            processTelemetryData(telemetry);
        }, 1000);
    }

    function simulateCommandResponse(command) {
        // Simulate different responses based on command
        setTimeout(() => {
            switch (command) {
                case 'PING':
                    processCommandResponse({ success: true, message: 'Pong! Conexión OK' });
                    break;

                case 'GET_STATUS':
                    processStatusData({
                        rocketState: appState.lastTelemetry.rocketState || 0x02,
                        batteryVoltage: appState.lastTelemetry.batteryVoltage || 4.1,
                        batteryPercentage: 85,
                        rssi: appState.lastTelemetry.rssi || -65,
                        snr: appState.lastTelemetry.snr || 10
                    });
                    processCommandResponse({ success: true });
                    break;

                case 'WAKE_UP_COMMAND':
                    // Change state to READY
                    appState.lastTelemetry.rocketState = 0x03; // READY
                    updateRocketStateDisplay(0x03);
                    processCommandResponse({ success: true, message: 'Cohete despertado' });
                    break;

                case 'ARM_ROCKET':
                    // Change state to ARMED
                    appState.lastTelemetry.rocketState = 0x04; // ARMED
                    updateRocketStateDisplay(0x04);
                    processCommandResponse({ success: true, message: 'Cohete armado' });
                    break;

                case 'LAUNCH_COMMAND':
                    // Simulate launch sequence
                    processCommandResponse({ success: true, message: 'Iniciando secuencia de lanzamiento' });

                    // Change state to COUNTDOWN
                    appState.lastTelemetry.rocketState = 0x05; // COUNTDOWN
                    updateRocketStateDisplay(0x05);

                    // After countdown, start flight
                    setTimeout(() => {
                        if (!appState.connected) return;

                        appState.lastTelemetry.rocketState = 0x10; // POWERED_FLIGHT
                        updateRocketStateDisplay(0x10);

                        // Start simulation
                        appState.missionStartTime = Date.now();
                        flightPhase = 'powered-flight';

                        logToConsole('¡¡¡DESPEGUE!!!', 'warning');
                        playAlertSound();
                    }, 5000);
                    break;

                case 'ABORT_COMMAND':
                    processCommandResponse({ success: true, message: 'Misión abortada' });

                    // Reset simulation
                    flightPhase = 'pre-launch';
                    appState.lastTelemetry.rocketState = 0x02; // IDLE
                    updateRocketStateDisplay(0x02);
                    break;

                case 'CALIBRATE_SENSORS':
                    processCommandResponse({ success: true, message: 'Sensores calibrados' });
                    break;

                case 'RUN_DIAGNOSTICS':
                    processCommandResponse({ success: true, message: 'Diagnóstico iniciado' });

                    // Simulate diagnostic results
                    setTimeout(() => {
                        processDiagnosticData({
                            tests: [
                                { name: 'Barómetro', passed: true },
                                { name: 'IMU', passed: true },
                                { name: 'GPS', passed: true },
                                { name: 'Paracaídas', passed: true },
                                { name: 'Flash Storage', passed: true },
                                { name: 'Batería', passed: true },
                                { name: 'Radio', passed: true }
                            ]
                        });
                    }, 3000);
                    break;

                case 'FORCE_DEPLOY_PARACHUTE':
                    processCommandResponse({ success: true, message: '¡Paracaídas desplegado!' });

                    // Change state to PARACHUTE_DESCENT
                    appState.lastTelemetry.rocketState = 0x14; // PARACHUTE_DESCENT
                    updateRocketStateDisplay(0x14);
                    flightPhase = 'parachute';
                    break;

                default:
                    processCommandResponse({ success: false, message: 'Comando desconocido' });
                    break;
            }
        }, 500 + Math.random() * 500); // Random delay to simulate transmission
    }

    // Initialize the dashboard
    init();

    // Update mission time periodically
    setInterval(updateMissionTime, 1000);

    // Update graph timespan periodically
    setInterval(updateGraphTimespan, 5000);
});