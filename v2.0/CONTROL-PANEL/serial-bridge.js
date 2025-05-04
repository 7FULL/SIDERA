const SerialBridge = (function() {
    // Variables privadas
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

    // Configuración predeterminada
    const defaultOptions = {
        baudRate: 115200,
    };

    // Comandos del protocolo
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

    // Verificar si Web Serial API está disponible
    const isSupported = () => {
        return 'serial' in navigator;
    };

    // Inicializar el bridge
    const init = (options = {}) => {
        // Fusionar opciones con los valores predeterminados
        const config = { ...defaultOptions, ...options };

        // Guardar callbacks
        if (options.onPortsDiscovered) callbacks.onPortsDiscovered = options.onPortsDiscovered;
        if (options.onConnect) callbacks.onConnect = options.onConnect;
        if (options.onDisconnect) callbacks.onDisconnect = options.onDisconnect;
        if (options.onData) callbacks.onData = options.onData;
        if (options.onError) callbacks.onError = options.onError;

        // Verificar soporte
        if (!isSupported()) {
            console.error("Web Serial API no está soportada en este navegador");
            callbacks.onError("Web Serial API no soportada");
            return false;
        }

        // Escuchar conexión/desconexión de puertos
        navigator.serial.addEventListener('connect', (e) => {
            console.log('Dispositivo conectado:', e);
            listPorts();
        });

        navigator.serial.addEventListener('disconnect', (e) => {
            console.log('Dispositivo desconectado:', e);
            if (port && e.target === port) {
                disconnect();
            }
            listPorts();
        });

        return true;
    };

    // Listar puertos disponibles
    const listPorts = async () => {
        if (!isSupported()) {
            callbacks.onError("Web Serial API no soportada");
            return;
        }

        try {
            const ports = await navigator.serial.getPorts();
            const portInfos = ports.map(port => {
                // Intentar obtener información del puerto
                let info = "Puerto desconocido";
                if (port.getInfo) {
                    const portInfo = port.getInfo();
                    if (portInfo.usbVendorId) {
                        info = `VID: ${portInfo.usbVendorId.toString(16)}, PID: ${portInfo.usbProductId.toString(16)}`;
                    }
                }
                return info;
            });

            callbacks.onPortsDiscovered(portInfos.length > 0 ? portInfos : ['No hay puertos disponibles']);
        } catch (error) {
            console.error('Error al listar puertos:', error);
            callbacks.onError(`Error al listar puertos: ${error.message}`);
        }
    };

    // Solicitar puerto
    const requestPort = async () => {
        if (!isSupported()) {
            callbacks.onError("Web Serial API no soportada");
            return null;
        }

        try {
            return await navigator.serial.requestPort();
        } catch (error) {
            console.error('Error al solicitar puerto:', error);
            callbacks.onError(`Error al solicitar puerto: ${error.message}`);
            return null;
        }
    };

    // Conectar a un puerto específico o solicitar uno
    const connect = async (requestedPort) => {
        if (!isSupported()) {
            callbacks.onError("Web Serial API no soportada");
            return false;
        }

        try {
            // Si no tenemos un puerto, solicitamos uno
            if (!requestedPort) {
                port = await requestPort();
                if (!port) return false;
            } else {
                // Intentar encontrar el puerto solicitado en la lista de puertos
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

            // Abrir puerto
            await port.open({ baudRate: defaultOptions.baudRate });

            // Configurar lector y escritor
            const textDecoder = new TextDecoderStream();
            const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
            reader = textDecoder.readable.getReader();

            const textEncoder = new TextEncoderStream();
            const writableStreamClosed = textEncoder.readable.pipeTo(port.writable);
            writer = textEncoder.writable.getWriter();

            // Iniciar tarea de lectura
            startReading();

            // Notificar conexión exitosa
            callbacks.onConnect();
            return true;
        } catch (error) {
            console.error('Error al conectar:', error);
            callbacks.onError(`Error al conectar: ${error.message}`);
            await disconnect();
            return false;
        }
    };

    // Desconectar
    const disconnect = async () => {
        // Detener la lectura
        if (readingTask) {
            readingTask.cancel();
            readingTask = null;
        }

        // Cerrar escritor
        if (writer) {
            try {
                await writer.close();
            } catch (error) {
                console.error('Error al cerrar escritor:', error);
            }
            writer = null;
        }

        // Cerrar lector
        if (reader) {
            try {
                await reader.cancel();
                await reader.releaseLock();
            } catch (error) {
                console.error('Error al cerrar lector:', error);
            }
            reader = null;
        }

        // Cerrar puerto
        if (port) {
            try {
                await port.close();
            } catch (error) {
                console.error('Error al cerrar puerto:', error);
            }
            port = null;
        }

        // Notificar desconexión
        callbacks.onDisconnect();
        return true;
    };

    // Iniciar tarea de lectura
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

                    // Añadir datos al buffer
                    buffer += value;

                    // Procesar líneas completas
                    const lines = buffer.split('\n');
                    buffer = lines.pop(); // La última línea queda en el buffer si no está completa

                    for (const line of lines) {
                        const trimmedLine = line.trim();
                        if (trimmedLine) {
                            processReceivedData(trimmedLine);
                        }
                    }
                }
            } catch (error) {
                console.error('Error al leer del puerto:', error);
                callbacks.onError(`Error al leer del puerto: ${error.message}`);
                disconnect();
            }
        })();
    };

    // Procesar datos recibidos
    const processReceivedData = (data) => {
        console.log("Datos recibidos:", data);

        // Intentar parsear como JSON primero
        try {
            //If it starts with __telemetry__ then parse it as JSON
            if (data.startsWith('__telemetry__')) {
                data = data.substring('__telemetry__'.length);
            }else{
                // Si no es JSON, continuar
                return;
            }

            console.log("Parsing JSON data:", data);

            const jsonData = JSON.parse(data);
            callbacks.onData(jsonData);
            return;
        } catch (e) {
            // No es JSON, seguir procesando
        }

        return;

        // Procesar respuestas del formato del PCB CONTROL PANEL
        if (data.startsWith('Received telemetry')) {
            // Extraer datos de telemetría (formato: "Received telemetry [Alt: 123.4m, V: 45.6m/s]")
            const altMatch = data.match(/Altitude: ([\d.]+)m/);
            const speedMatch = data.match(/Vertical Speed: ([\d.-]+)m\/s/);
            const accelMatch = data.match(/Acceleration: ([\d.-]+)m\/s²/);
            const tempMatch = data.match(/Temperature: ([\d.-]+)°C/);
            const pressureMatch = data.match(/Pressure: ([\d.-]+)hPa/);
            const batteryMatch = data.match(/Battery: ([\d.-]+)V/);
            const gpsMatch = data.match(/GPS: Lat: ([\d.-]+), Lon: ([\d.-]+)/);
            const pitchMatch = data.match(/Pitch: ([\d.-]+)°/);
            const rollMatch = data.match(/Roll: ([\d.-]+)°/);
            const yawMatch = data.match(/Yaw: ([\d.-]+)°/);
            const rssiMatch = data.match(/RSSI: ([-\d]+)dBm/);
            const snrMatch = data.match(/SNR: ([-\d.]+)dB/);
            const stateMatch = data.match(/State: ([\w\s]+)/);

            if (altMatch && speedMatch) {
                const telemetryData = {
                    type: 'telemetry',
                    data: {
                        timestamp: Date.now(),
                        altitude: parseFloat(altMatch[1]),
                        verticalSpeed: parseFloat(speedMatch[1]),
                        acceleration: accelMatch ? parseFloat(accelMatch[1]) : 0,
                        temperature: tempMatch ? parseFloat(tempMatch[1]) : 0,
                        pressure: pressureMatch ? parseFloat(pressureMatch[1]) : 0,
                        batteryVoltage: batteryMatch ? parseFloat(batteryMatch[1]) : 4.0,
                        gpsSatellites: gpsMatch ? parseInt(gpsMatch[1]) : 0,
                        gpsLatitude: gpsMatch ? parseFloat(gpsMatch[1]) : 0,
                        gpsLongitude: gpsMatch ? parseFloat(gpsMatch[2]) : 0,
                        gpsAltitude: gpsMatch ? parseFloat(gpsMatch[3]) : 0,
                        pitch: pitchMatch ? parseFloat(pitchMatch[1]) : 0,
                        roll: rollMatch ? parseFloat(rollMatch[1]) : 0,
                        yaw: yawMatch ? parseFloat(yawMatch[1]) : 0,
                        rssi: rssiMatch ? parseInt(rssiMatch[1]) : -70,
                        snr: snrMatch ? parseFloat(snrMatch[1]) : 10,
                        rocketState: stateMatch ? stateMatch[1] : 'UNKNOWN',
                    }
                };

                callbacks.onData(JSON.stringify(telemetryData));
            }
        }
        // Mejorado: Detectar respuestas a comandos Lanzar/Abortar
        else if (data.includes('Are you sure?') || data.includes('WARNING:')) {
            // No hacer nada aquí, esto es la confirmación que está esperando el "y"
            console.log("Confirmación solicitada para comando crítico");
        }
        // Procesar confirmaciones de comandos enviados
        else if (data.includes('Launching') || data.includes('DESPEGUE') ||
            data.includes('iniciando secuencia') || data.includes('cuenta regresiva')) {
            // Confirmación de lanzamiento
            const responseData = {
                type: 'command_response',
                data: {
                    success: true,
                    message: 'Lanzamiento iniciado'
                }
            };
            callbacks.onData(JSON.stringify(responseData));
        }
        else if (data.includes('abortada') || data.includes('Aborting') ||
            data.includes('Abort command')) {
            // Confirmación de aborto
            const responseData = {
                type: 'command_response',
                data: {
                    success: true,
                    message: 'Misión abortada'
                }
            };
            callbacks.onData(JSON.stringify(responseData));
        }
        else if (data.includes('SUCCESS') || data.includes('executed') ||
            data.includes('successfully') || data.includes('OK') ||
            data.includes('received') || data.includes('pong') || data.includes('Pong')) {
            // Respuesta de comando exitoso
            const responseData = {
                type: 'command_response',
                data: {
                    success: true,
                    message: data
                }
            };
            callbacks.onData(JSON.stringify(responseData));
        }
        else if (data.includes('ERROR') || data.includes('FAILED') ||
            data.includes('error') || data.includes('failed')) {
            // Respuesta de comando fallido
            const responseData = {
                type: 'command_response',
                data: {
                    success: false,
                    message: data
                }
            };
            callbacks.onData(JSON.stringify(responseData));
        }
        else if (data.includes('State:')) {
            // Datos de estado del cohete
            let stateCode = 0x02; // IDLE por defecto

            if (data.includes('INITIALIZING')) stateCode = 0x01;
            else if (data.includes('IDLE')) stateCode = 0x02;
            else if (data.includes('READY')) stateCode = 0x03;
            else if (data.includes('ARMED')) stateCode = 0x04;
            else if (data.includes('COUNTDOWN')) stateCode = 0x05;
            else if (data.includes('POWERED FLIGHT')) stateCode = 0x10;
            else if (data.includes('COASTING')) stateCode = 0x11;
            else if (data.includes('APOGEE')) stateCode = 0x12;
            else if (data.includes('DESCENT')) stateCode = 0x13;
            else if (data.includes('PARACHUTE')) stateCode = 0x14;
            else if (data.includes('LANDED')) stateCode = 0x20;
            else if (data.includes('ERROR')) stateCode = 0xE0;

            // Extraer datos de batería
            const batteryMatch = data.match(/Battery: ([\d.]+)V/);
            const batteryVoltage = batteryMatch ? parseFloat(batteryMatch[1]) : 4.0;

            // Extraer datos de señal
            const rssiMatch = data.match(/RSSI ([-\d]+)dBm/);
            const snrMatch = data.match(/SNR ([-\d.]+)dB/);
            const rssi = rssiMatch ? parseInt(rssiMatch[1]) : -70;
            const snr = snrMatch ? parseFloat(snrMatch[1]) : 10;

            const statusData = {
                type: 'status',
                data: {
                    rocketState: stateCode,
                    batteryVoltage: batteryVoltage,
                    batteryPercentage: ((batteryVoltage - 3.0) / 1.2) * 100,
                    rssi: rssi,
                    snr: snr
                }
            };

            callbacks.onData(JSON.stringify(statusData));
        }
        // Como último recurso para los comandos críticos que pudieron no ser detectados
        else if (lastCommandSent &&
            (lastCommandSent === 'LAUNCH_COMMAND' ||
                lastCommandSent === 'ABORT_COMMAND' ||
                lastCommandSent === 'FORCE_DEPLOY_PARACHUTE') &&
            data.includes('y')) {
            // El usuario ha confirmado un comando crítico
            setTimeout(() => {
                // Generar una respuesta sintética
                const responseData = {
                    type: 'command_response',
                    data: {
                        success: true,
                        message: `Comando ${lastCommandSent} ejecutado con éxito`
                    }
                };
                callbacks.onData(JSON.stringify(responseData));
                lastCommandSent = null;
            }, 500);
        }
        else {
            // Otros datos, enviar como texto plano
            callbacks.onData(data);
        }
    };

    // Enviar comando
    const sendCommand = async (command) => {
        if (!writer || !port) {
            callbacks.onError("No conectado");
            return false;
        }

        try {
            let commandChar = '';
            lastCommandSent = command.command;

            // Convertir comando a carácter correspondiente según el protocolo del PCB CONTROL PANEL
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
                    commandChar = 'h'; // Ayuda por defecto
            }

            // Enviar comando
            console.log(`Enviando comando: ${command.command} (char: ${commandChar})`);
            await writer.write(commandChar);

            // Para comandos que requieren confirmación, enviar 'y' después
            if (command.command === 'LAUNCH_COMMAND' ||
                command.command === 'FORCE_DEPLOY_PARACHUTE' ||
                command.command === 'ABORT_COMMAND') {

                // Pequeña pausa para que el panel de control muestre el mensaje de confirmación
                await new Promise(resolve => setTimeout(resolve, 500));

                // Enviar confirmación
                console.log("Enviando confirmación 'y'");
                await writer.write('y');
            }

            return true;
        } catch (error) {
            console.error('Error al enviar comando:', error);
            callbacks.onError(`Error al enviar comando: ${error.message}`);
            return false;
        }
    };

    // API pública
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