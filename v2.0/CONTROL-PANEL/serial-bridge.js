const SerialBridge = (function() {
    // Private variables
    let port = null;
    let reader = null;
    let writer = null;
    let readingTask = null;
    let lastCommandSent = null;
    let callbacks = {
        onPortsDiscovered: () => {},
        onConnect: () => {},
        onDisconnect: () => {},
        onData: () => {},
        onError: () => {}
    };

    // Default configuration
    const defaultOptions = {
        baudRate: 115200,
    };

    // Protocol commands
    const COMMANDS = {
        PING: 0x01,
        GET_STATUS: 0x02,
        WAKE_UP_COMMAND: 0x03,
        ABORT_COMMAND: 0x04,
        CALIBRATE_SENSORS: 0x05,
        RUN_DIAGNOSTICS: 0x06,
        LAUNCH_COMMAND: 0x08,
        FORCE_DEPLOY_PARACHUTE: 0x09
    };

    // Check if Web Serial API is available
    const isSupported = () => {
        return 'serial' in navigator;
    };

    // Initialize the bridge
    const init = (options = {}) => {
        // Merge options with default values
        const config = { ...defaultOptions, ...options };

        // Save callbacks
        if (options.onPortsDiscovered) callbacks.onPortsDiscovered = options.onPortsDiscovered;
        if (options.onConnect) callbacks.onConnect = options.onConnect;
        if (options.onDisconnect) callbacks.onDisconnect = options.onDisconnect;
        if (options.onData) callbacks.onData = options.onData;
        if (options.onError) callbacks.onError = options.onError;

        // Check support
        if (!isSupported()) {
            console.error("Web Serial API is not supported in this browser");
            callbacks.onError("Web Serial API not supported");
            return false;
        }

        // Listen for port connection/disconnection
        navigator.serial.addEventListener('connect', (e) => {
            console.log('Device connected:', e);
            listPorts();
        });

        navigator.serial.addEventListener('disconnect', (e) => {
            console.log('Device disconnected:', e);
            if (port && e.target === port) {
                disconnect();
            }
            listPorts();
        });

        return true;
    };

    // List available ports
    const listPorts = async () => {
        if (!isSupported()) {
            callbacks.onError("Web Serial API not supported");
            return;
        }

        try {
            const ports = await navigator.serial.getPorts();
            const portInfos = ports.map(port => {
                // Try to get port information
                let info = "Unknown port";
                if (port.getInfo) {
                    const portInfo = port.getInfo();
                    if (portInfo.usbVendorId) {
                        info = `VID: ${portInfo.usbVendorId.toString(16)}, PID: ${portInfo.usbProductId.toString(16)}`;
                    }
                }
                return info;
            });

            callbacks.onPortsDiscovered(portInfos.length > 0 ? portInfos : ['No ports available']);
        } catch (error) {
            console.error('Error listing ports:', error);
            callbacks.onError(`Error listing ports: ${error.message}`);
        }
    };

    // Request port
    const requestPort = async () => {
        if (!isSupported()) {
            callbacks.onError("Web Serial API not supported");
            return null;
        }

        try {
            return await navigator.serial.requestPort();
        } catch (error) {
            console.error('Error requesting port:', error);
            callbacks.onError(`Error requesting port: ${error.message}`);
            return null;
        }
    };

    // Connect to a specific port or request one
    const connect = async (requestedPort) => {
        if (!isSupported()) {
            callbacks.onError("Web Serial API not supported");
            return false;
        }

        try {
            // If we don't have a port, request one
            if (!requestedPort) {
                port = await requestPort();
                if (!port) return false;
            } else {
                // Try to find the requested port in the port list
                const ports = await navigator.serial.getPorts();
                const matchedPort = ports.find(p => {
                    const info = p.getInfo ? p.getInfo() : {};
                    return JSON.stringify(info) === requestedPort;
                });

                if (matchedPort) {
                    port = matchedPort;
                } else {
                    port = await requestPort();
                    if (!port) return false;
                }
            }

            // Open port
            await port.open({ baudRate: defaultOptions.baudRate });

            // Set up reader and writer
            const textDecoder = new TextDecoderStream();
            const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
            reader = textDecoder.readable.getReader();

            const textEncoder = new TextEncoderStream();
            const writableStreamClosed = textEncoder.readable.pipeTo(port.writable);
            writer = textEncoder.writable.getWriter();

            // Start reading task
            startReading();

            // Notify successful connection
            callbacks.onConnect();
            return true;
        } catch (error) {
            console.error('Error connecting:', error);
            callbacks.onError(`Error connecting: ${error.message}`);
            await disconnect();
            return false;
        }
    };

    // Disconnect
    const disconnect = async () => {
        // Stop reading
        if (readingTask) {
            readingTask.cancel();
            readingTask = null;
        }

        // Close writer
        if (writer) {
            try {
                await writer.close();
            } catch (error) {
                console.error('Error closing writer:', error);
            }
            writer = null;
        }

        // Close reader
        if (reader) {
            try {
                await reader.cancel();
                await reader.releaseLock();
            } catch (error) {
                console.error('Error closing reader:', error);
            }
            reader = null;
        }

        // Close port
        if (port) {
            try {
                await port.close();
            } catch (error) {
                console.error('Error closing port:', error);
            }
            port = null;
        }

        // Notify disconnection
        callbacks.onDisconnect();
        return true;
    };

    // Start reading task
    const startReading = () => {
        if (!reader) return;

        let buffer = '';

        readingTask = (async () => {
            try {
                while (true) {
                    const { value, done } = await reader.read();
                    if (done) {
                        break;
                    }

                    // Add data to buffer
                    buffer += value;

                    // Process complete lines
                    const lines = buffer.split('\n');
                    buffer = lines.pop(); // The last line stays in the buffer if not complete

                    for (const line of lines) {
                        const trimmedLine = line.trim();
                        if (trimmedLine) {
                            processReceivedData(trimmedLine);
                        }
                    }
                }
            } catch (error) {
                console.error('Error reading from port:', error);
                callbacks.onError(`Error reading from port: ${error.message}`);
                disconnect();
            }
        })();
    };

    // Process received data
    const processReceivedData = (data) => {
        console.log("Data received:", data);

        // Try to parse as JSON first
        try {
            // If it starts with __telemetry__ then parse it as JSON
            if (data.startsWith('__tl__')) {
                data = data.substring('__tl__'.length);
            }else{
                // If it's not JSON, continue
                return;
            }

            console.log("Parsing JSON data:", data);

            // const jsonData = JSON.parse(data);
            callbacks.onData(data);
            return;
        } catch (e) {
            // Not JSON, continue processing
        }

        return;
    };

    // Send command
    const sendCommand = async (command) => {
        if (!writer || !port) {
            callbacks.onError("Not connected");
            return false;
        }

        try {
            let commandChar = '';
            lastCommandSent = command.command;

            // Convert command to corresponding character according to PCB CONTROL PANEL protocol
            switch (command.command) {
                case 'PING':
                    commandChar = 'p';
                    break;
                case 'GET_STATUS':
                    commandChar = 's';
                    break;
                case 'WAKE_UP_COMMAND':
                    commandChar = 'l';
                    break;
                case 'ABORT_COMMAND':
                    commandChar = 'x';
                    break;
                case 'CALIBRATE_SENSORS':
                    commandChar = 'c';
                    break;
                case 'RUN_DIAGNOSTICS':
                    commandChar = 'r';
                    break;
                case 'LAUNCH_COMMAND':
                    commandChar = 'l';
                    break;
                case 'FORCE_DEPLOY_PARACHUTE':
                    commandChar = 'e';
                    break;
                default:
                    commandChar = 'h'; // Help by default
            }

            // Send command
            console.log(`Sending command: ${command.command} (char: ${commandChar})`);
            await writer.write(commandChar);

            // For commands that require confirmation, send 'y' after
            if (command.command === 'LAUNCH_COMMAND' ||
                command.command === 'FORCE_DEPLOY_PARACHUTE' ||
                command.command === 'ABORT_COMMAND') {

                // Small pause so control panel shows confirmation message
                await new Promise(resolve => setTimeout(resolve, 500));

                // Send confirmation
                console.log("Sending confirmation 'y'");
                await writer.write('y');
            }

            return true;
        } catch (error) {
            console.error('Error sending command:', error);
            callbacks.onError(`Error sending command: ${error.message}`);
            return false;
        }
    };

    // Public API
    return {
        init,
        isSupported,
        listPorts,
        requestPort,
        connect,
        disconnect,
        sendCommand
    };
})();