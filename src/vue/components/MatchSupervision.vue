<template>
  <div class="match-supervision-container">
    <!-- En-tête de contrôle de match -->
    <div class="match-header">
      <div class="match-controls">
        <button class="btn btn-success" @click="startMatch" :disabled="matchRunning || !systemReady">
          ▶️ Démarrer Match
        </button>
        <button class="btn btn-warning" @click="pauseMatch" :disabled="!matchRunning">
          ⏸️ Pause
        </button>
        <button class="btn btn-danger" @click="stopMatch" :disabled="!matchRunning">
          ⏹️ Arrêter
        </button>
        <button class="btn btn-outline" @click="resetMatch">
          🔄 Reset
        </button>
      </div>
      
      <div class="match-status">
        <div class="status-indicator" :class="systemStatusClass">
          <div class="status-dot animate-pulse" v-if="systemReady"></div>
          <div class="status-dot" v-else></div>
          {{ systemStatusText }}
        </div>
        <div class="match-timer" :class="timerClass">
          {{ formatTime(matchTimeRemaining) }}
        </div>
        <div class="match-score">
          Score: {{ currentScore }}
        </div>
      </div>
    </div>

    <!-- Section principale - 3 panneaux -->
    <div class="supervision-layout">
      <!-- Panneau Gauche - Carte RViz et Diagnostic -->
      <div class="left-panel">
        <!-- Intégration RViz / Carte 2D -->
        <div class="card rviz-container">
          <div class="card-header">
            <h3 class="card-title">Carte ROS / RViz</h3>
            <div class="rviz-controls">
              <button class="btn btn-outline btn-sm" @click="refreshRViz">🔄</button>
              <button class="btn btn-outline btn-sm" @click="centerRobot">🎯</button>
              <button class="btn btn-outline btn-sm" @click="toggleRVizMode">
                {{ rvizMode === 'map' ? '🗺️' : '📷' }}
              </button>
            </div>
          </div>
          <div class="rviz-display">
            <div v-if="rvizConnected" class="rviz-frame">
              <!-- Ici s'affichera RViz via WebRTC ou image stream -->
              <iframe 
                v-if="rvizMode === 'web'"
                :src="rvizWebUrl" 
                width="100%" 
                height="100%"
                frameborder="0">
              </iframe>
              <div v-else class="rviz-placeholder">
                <div class="placeholder-icon">🗺️</div>
                <p>Carte ROS en cours de chargement...</p>
                <small>Mode: {{ rvizMode }}</small>
              </div>
            </div>
            <div v-else class="rviz-disconnected">
              <div class="disconnect-icon">❌</div>
              <p>RViz non connecté</p>
              <button class="btn btn-primary btn-sm" @click="connectRViz">
                Connecter RViz
              </button>
            </div>
          </div>
        </div>

        <!-- Système de Ping/Diagnostic -->
        <div class="card ping-system">
          <div class="card-header">
            <h3 class="card-title">Diagnostic Système</h3>
            <div class="ping-controls">
              <button class="btn btn-outline btn-sm" @click="pingAllSystems">
                🔍 Ping All
              </button>
              <button class="btn btn-outline btn-sm" @click="runFullDiagnostic">
                ⚙️ Diagnostic
              </button>
            </div>
          </div>
          <div class="ping-grid">
            <div 
              v-for="component in systemComponents" 
              :key="component.name"
              class="ping-item"
              :class="component.status"
              @click="pingComponent(component)"
            >
              <div class="ping-icon">{{ component.icon }}</div>
              <div class="ping-info">
                <div class="ping-name">{{ component.name }}</div>
                <div class="ping-details">{{ component.details }}</div>
              </div>
              <div class="ping-status">
                <div class="ping-latency" v-if="component.latency">
                  {{ component.latency }}ms
                </div>
                <div class="ping-indicator" :class="component.status"></div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- Panneau Central - Terminal et Logs -->
      <div class="center-panel">
        <!-- Terminal ROS intégré -->
        <div class="card terminal-container">
          <div class="card-header">
            <h3 class="card-title">Terminal ROS</h3>
            <div class="terminal-controls">
              <button class="btn btn-outline btn-sm" @click="clearTerminal">🗑️</button>
              <button class="btn btn-outline btn-sm" @click="saveTerminalOutput">💾</button>
              <select v-model="selectedTerminalMode" class="terminal-mode-select">
                <option value="ros">ROS Commands</option>
                <option value="system">System Shell</option>
                <option value="logs">ROS Logs</option>
              </select>
            </div>
          </div>
          <div class="terminal-display">
            <div class="terminal-output" ref="terminalOutput">
              <div 
                v-for="line in terminalLines" 
                :key="line.id"
                class="terminal-line"
                :class="line.type"
              >
                <span class="terminal-timestamp">{{ formatTerminalTime(line.timestamp) }}</span>
                <span class="terminal-prefix">{{ line.prefix }}</span>
                <span class="terminal-content">{{ line.content }}</span>
              </div>
            </div>
            <div class="terminal-input">
              <span class="terminal-prompt">{{ terminalPrompt }}</span>
              <input 
                ref="terminalInputField"
                v-model="currentCommand"
                @keydown.enter="executeCommand"
                @keydown.up="previousCommand"
                @keydown.down="nextCommand"
                class="terminal-field"
                :placeholder="terminalPlaceholder"
              />
            </div>
          </div>
        </div>

        <!-- Logs temps réel -->
        <div class="card logs-container">
          <div class="card-header">
            <h3 class="card-title">Logs ROS Temps Réel</h3>
            <div class="logs-controls">
              <select v-model="selectedLogLevel" class="log-level-filter">
                <option value="all">Tous</option>
                <option value="debug">Debug</option>
                <option value="info">Info</option>
                <option value="warning">Warning</option>
                <option value="error">Error</option>
                <option value="fatal">Fatal</option>
              </select>
              <button class="btn btn-outline btn-sm" @click="clearLogs">🗑️</button>
              <button class="btn btn-outline btn-sm" @click="exportLogs">📤</button>
            </div>
          </div>
          <div class="logs-display">
            <div 
              v-for="log in filteredLogs" 
              :key="log.id"
              class="log-entry"
              :class="log.level"
            >
              <div class="log-timestamp">{{ formatLogTime(log.timestamp) }}</div>
              <div class="log-level">{{ log.level }}</div>
              <div class="log-node">{{ log.node }}</div>
              <div class="log-message">{{ log.message }}</div>
            </div>
          </div>
        </div>
      </div>

      <!-- Panneau Droit - Monitoring et Actions -->
      <div class="right-panel">
        <!-- État des nœuds en temps réel -->
        <div class="card nodes-monitor">
          <div class="card-header">
            <h3 class="card-title">Nœuds ROS Actifs</h3>
            <div class="nodes-controls">
              <button class="btn btn-outline btn-sm" @click="refreshNodes">🔄</button>
              <div class="auto-refresh-toggle">
                <input type="checkbox" v-model="autoRefreshNodes" id="autoRefresh">
                <label for="autoRefresh">Auto</label>
              </div>
            </div>
          </div>
          <div class="nodes-list">
            <div 
              v-for="node in activeNodes" 
              :key="node.name"
              class="node-item"
              :class="node.status"
            >
              <div class="node-icon">{{ getNodeIcon(node.type) }}</div>
              <div class="node-info">
                <div class="node-name">{{ node.name }}</div>
                <div class="node-topics">{{ node.topics?.length || 0 }} topics</div>
              </div>
              <div class="node-metrics">
                <div class="node-hz">{{ node.hz || 0 }}Hz</div>
                <div class="node-status-dot" :class="node.status"></div>
              </div>
            </div>
          </div>
        </div>

        <!-- Actions rapides -->
        <div class="card quick-actions">
          <div class="card-header">
            <h3 class="card-title">Actions Rapides</h3>
          </div>
          <div class="actions-grid">
            <button class="action-btn" @click="emergencyStop">
              🚨 ARRÊT D'URGENCE
            </button>
            <button class="action-btn" @click="robotGoHome">
              🏠 Robot à la base
            </button>
            <button class="action-btn" @click="calibrateAll">
              ⚙️ Calibrer tout
            </button>
            <button class="action-btn" @click="requestManualReset">
              🔄 Reset manuel (-10pts)
            </button>
            <button class="action-btn" @click="pauseConveyor">
              ⏸️ Pause convoyeur
            </button>
            <button class="action-btn" @click="restartArm">
              🥾 Restart bras
            </button>
          </div>
        </div>

        <!-- Métriques système -->
        <div class="card system-metrics">
          <div class="card-header">
            <h3 class="card-title">Métriques Système</h3>
          </div>
          <div class="metrics-display">
            <div class="metric-item">
              <div class="metric-label">CPU Usage:</div>
              <div class="metric-value">{{ systemMetrics.cpu }}%</div>
              <div class="metric-bar">
                <div class="metric-fill" :style="{ width: systemMetrics.cpu + '%' }"></div>
              </div>
            </div>
            <div class="metric-item">
              <div class="metric-label">RAM Usage:</div>
              <div class="metric-value">{{ systemMetrics.ram }}%</div>
              <div class="metric-bar">
                <div class="metric-fill" :style="{ width: systemMetrics.ram + '%' }"></div>
              </div>
            </div>
            <div class="metric-item">
              <div class="metric-label">Messages/sec:</div>
              <div class="metric-value">{{ systemMetrics.messagesPerSec }}</div>
            </div>
            <div class="metric-item">
              <div class="metric-label">Network:</div>
              <div class="metric-value">{{ systemMetrics.network }}KB/s</div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { ref, computed, onMounted, onUnmounted, nextTick } from 'vue'
import ROSService from '../services/ROSService.js'

export default {
  name: 'MatchSupervision',
  setup() {
    // État du match
    const matchRunning = ref(false)
    const matchTimeRemaining = ref(300) // 5 minutes
    const currentScore = ref(0)
    const systemReady = ref(false)
    
    // RViz et carte
    const rvizConnected = ref(false)
    const rvizMode = ref('web') // 'web', 'stream', 'static'
    const rvizWebUrl = ref('http://localhost:8080') // URL du serveur RViz web
    
    // Terminal
    const terminalLines = ref([
      { id: 1, timestamp: Date.now(), type: 'info', prefix: 'roslaunch', content: 'TEKBOT system started successfully' },
      { id: 2, timestamp: Date.now() + 1000, type: 'success', prefix: 'rosnode', content: 'All nodes are running' }
    ])
    const currentCommand = ref('')
    const commandHistory = ref(['rosnode list', 'rostopic list', 'rosservice list'])
    const commandHistoryIndex = ref(-1)
    const selectedTerminalMode = ref('ros')
    
    // Logs
    const rosLogs = ref([
      { id: 1, timestamp: Date.now(), level: 'info', node: 'robot_node', message: 'Robot initialized successfully' },
      { id: 2, timestamp: Date.now() + 1000, level: 'warning', node: 'camera_node', message: 'Low light conditions detected' },
      { id: 3, timestamp: Date.now() + 2000, level: 'info', node: 'conveyor_node', message: 'Conveyor calibration complete' }
    ])
    const selectedLogLevel = ref('all')
    
    // Nœuds ROS
    const activeNodes = ref([
      { name: '/robot_node', type: 'robot', status: 'connected', topics: ['odom', 'cmd_vel'], hz: 10 },
      { name: '/conveyor_node', type: 'conveyor', status: 'connected', topics: ['sensor_data'], hz: 20 },
      { name: '/arm_node', type: 'arm', status: 'warning', topics: ['joint_states'], hz: 5 },
      { name: '/camera_node', type: 'sensor', status: 'disconnected', topics: [], hz: 0 }
    ])
    const autoRefreshNodes = ref(true)
    
    // Composants système pour ping
    const systemComponents = ref([
      { name: 'Robot Mobile', icon: '🤖', status: 'connected', details: 'Robomaster X3', latency: 12 },
      { name: 'Convoyeur', icon: '🏢', status: 'connected', details: 'ESP32 + Moteur', latency: 8 },
      { name: 'Bras Robotique', icon: '🥾', status: 'warning', details: 'Dofbot Jetson', latency: 45 },
      { name: 'Capteur LDR Front', icon: '👁️', status: 'connected', details: 'Analogique', latency: 5 },
      { name: 'Capteur LDR Back', icon: '👁️', status: 'connected', details: 'Analogique', latency: 7 },
      { name: 'Capteur Couleur', icon: '🌈', status: 'connected', details: 'RGB Sensor', latency: 15 },
      { name: 'Caméra QR', icon: '📷', status: 'disconnected', details: 'USB Camera', latency: null },
      { name: 'Bridge ROS', icon: '🌐', status: 'connected', details: 'WebSocket', latency: 3 }
    ])
    
    // Métriques système
    const systemMetrics = ref({
      cpu: 45,
      ram: 67,
      messagesPerSec: 145,
      network: 23.4
    })
    
    // Computed properties
    const systemStatusClass = computed(() => {
      const disconnected = systemComponents.value.filter(c => c.status === 'disconnected').length
      const warnings = systemComponents.value.filter(c => c.status === 'warning').length
      
      if (disconnected > 0) return 'status-error'
      if (warnings > 0) return 'status-warning'
      return 'status-connected'
    })
    
    const systemStatusText = computed(() => {
      const disconnected = systemComponents.value.filter(c => c.status === 'disconnected').length
      const warnings = systemComponents.value.filter(c => c.status === 'warning').length
      
      if (disconnected > 0) return `${disconnected} composant(s) déconnecté(s)`
      if (warnings > 0) return `${warnings} alerte(s) système`
      return 'Tous systèmes opérationnels'
    })
    
    const timerClass = computed(() => {
      if (matchTimeRemaining.value <= 30) return 'timer-critical'
      if (matchTimeRemaining.value <= 60) return 'timer-warning'
      return 'timer-normal'
    })
    
    const terminalPrompt = computed(() => {
      const prompts = {
        ros: 'ros2@tekbot:~$ ',
        system: 'tekbot@control:~$ ',
        logs: 'logs> '
      }
      return prompts[selectedTerminalMode.value] || 'tekbot:~$ '
    })
    
    const terminalPlaceholder = computed(() => {
      const placeholders = {
        ros: 'Ex: ros2 node list, ros2 topic echo /robot/odom',
        system: 'Ex: ps aux | grep ros, htop',
        logs: 'Ex: grep ERROR, tail -f /var/log/ros.log'
      }
      return placeholders[selectedTerminalMode.value] || 'Entrez une commande...'
    })
    
    const filteredLogs = computed(() => {
      if (selectedLogLevel.value === 'all') return rosLogs.value
      return rosLogs.value.filter(log => log.level === selectedLogLevel.value)
    })
    
    // Méthodes
    const formatTime = (seconds) => {
      const mins = Math.floor(seconds / 60)
      const secs = seconds % 60
      return `${mins}:${secs.toString().padStart(2, '0')}`
    }
    
    const formatTerminalTime = (timestamp) => {
      const date = new Date(timestamp)
      return date.toLocaleTimeString('fr-FR', { 
        hour: '2-digit', 
        minute: '2-digit', 
        second: '2-digit' 
      })
    }
    
    const formatLogTime = (timestamp) => {
      const date = new Date(timestamp)
      return date.toLocaleTimeString('fr-FR', { 
        hour: '2-digit', 
        minute: '2-digit', 
        second: '2-digit',
        fractionalSecondDigits: 3
      })
    }
    
    const getNodeIcon = (type) => {
      const icons = {
        robot: '🤖',
        conveyor: '🏢',
        arm: '🥾',
        sensor: '📷',
        system: '⚙️'
      }
      return icons[type] || '💻'
    }
    
    // Actions de match
    const startMatch = () => {
      if (!systemReady.value) return
      
      matchRunning.value = true
      addTerminalLine('info', 'match', 'Match démarré - Chronomètre activé')
      addLog('info', 'game_manager', 'Match started - 5 minutes timer activated')
      
      // Démarrer le timer
      startMatchTimer()
    }
    
    const pauseMatch = () => {
      matchRunning.value = false
      addTerminalLine('warning', 'match', 'Match mis en pause')
      addLog('warning', 'game_manager', 'Match paused by operator')
    }
    
    const stopMatch = () => {
      matchRunning.value = false
      addTerminalLine('error', 'match', 'Match arrêté')
      addLog('info', 'game_manager', 'Match stopped by operator')
    }
    
    const resetMatch = () => {
      if (confirm('Remettre à zéro le match et le score ?')) {
        matchRunning.value = false
        matchTimeRemaining.value = 300
        currentScore.value = 0
        addTerminalLine('info', 'match', 'Match remis à zéro')
        addLog('info', 'game_manager', 'Match reset - all systems reinitialized')
      }
    }
    
    // Actions RViz
    const refreshRViz = () => {
      addTerminalLine('info', 'rviz', 'Actualisation de la carte RViz...')
      // Simuler le rechargement
      setTimeout(() => {
        addTerminalLine('success', 'rviz', 'Carte RViz actualisée')
      }, 1500)
    }
    
    const centerRobot = () => {
      addTerminalLine('info', 'rviz', 'Centrage sur la position du robot')
      executeROSCommand('ros2 service call /rviz/center_on_robot std_srvs/srv/Trigger')
    }
    
    const toggleRVizMode = () => {
      const modes = ['web', 'stream', 'static']
      const currentIndex = modes.indexOf(rvizMode.value)
      rvizMode.value = modes[(currentIndex + 1) % modes.length]
      addTerminalLine('info', 'rviz', `Mode RViz changé: ${rvizMode.value}`)
    }
    
    const connectRViz = () => {
      addTerminalLine('info', 'rviz', 'Connexion à RViz...')
      setTimeout(() => {
        rvizConnected.value = true
        addTerminalLine('success', 'rviz', 'RViz connecté avec succès')
      }, 2000)
    }
    
    // Système de ping
    const pingComponent = async (component) => {
      addTerminalLine('info', 'ping', `Ping ${component.name}...`)
      
      // Simuler le ping
      setTimeout(() => {
        const success = Math.random() > 0.2
        if (success) {
          component.latency = Math.floor(Math.random() * 50) + 5
          component.status = component.latency > 100 ? 'warning' : 'connected'
          addTerminalLine('success', 'ping', `${component.name}: ${component.latency}ms`)
        } else {
          component.status = 'disconnected'
          component.latency = null
          addTerminalLine('error', 'ping', `${component.name}: Timeout`)
        }
      }, 1000)
    }
    
    const pingAllSystems = async () => {
      addTerminalLine('info', 'ping', 'Ping de tous les composants...')
      
      for (const component of systemComponents.value) {
        await new Promise(resolve => setTimeout(resolve, 500))
        await pingComponent(component)
      }
      
      addTerminalLine('success', 'ping', 'Ping complet terminé')
    }
    
    const runFullDiagnostic = () => {
      addTerminalLine('info', 'diagnostic', 'Lancement du diagnostic complet...')
      executeROSCommand('ros2 run tekbot_diagnostics full_system_check')
    }
    
    // Terminal
    const addTerminalLine = (type, prefix, content) => {
      const newLine = {
        id: Date.now() + Math.random(),
        timestamp: Date.now(),
        type,
        prefix,
        content
      }
      terminalLines.value.push(newLine)
      
      // Limiter à 200 lignes
      if (terminalLines.value.length > 200) {
        terminalLines.value = terminalLines.value.slice(-200)
      }
      
      // Auto-scroll
      nextTick(() => {
        const output = document.querySelector('.terminal-output')
        if (output) {
          output.scrollTop = output.scrollHeight
        }
      })
    }
    
    const executeCommand = () => {
      if (!currentCommand.value.trim()) return
      
      // Ajouter à l'historique
      commandHistory.value.push(currentCommand.value)
      commandHistoryIndex.value = commandHistory.value.length
      
      // Afficher la commande
      addTerminalLine('command', terminalPrompt.value, currentCommand.value)
      
      // Exécuter selon le mode
      if (selectedTerminalMode.value === 'ros') {
        executeROSCommand(currentCommand.value)
      } else if (selectedTerminalMode.value === 'system') {
        executeSystemCommand(currentCommand.value)
      } else {
        executeLogCommand(currentCommand.value)
      }
      
      currentCommand.value = ''
    }
    
    const executeROSCommand = (command) => {
      // Simuler l'exécution de commandes ROS
      setTimeout(() => {
        if (command.includes('node list')) {
          activeNodes.value.forEach(node => {
            addTerminalLine('output', '', node.name)
          })
        } else if (command.includes('topic list')) {
          addTerminalLine('output', '', '/robot/odom')
          addTerminalLine('output', '', '/robot/cmd_vel')
          addTerminalLine('output', '', '/conveyor/sensor_data')
          addTerminalLine('output', '', '/arm/joint_states')
        } else if (command.includes('service call')) {
          addTerminalLine('success', 'service', 'Service appelé avec succès')
        } else {
          addTerminalLine('output', '', `Commande ROS exécutée: ${command}`)
        }
      }, 500)
    }
    
    const executeSystemCommand = (command) => {
      setTimeout(() => {
        if (command.includes('ps aux')) {
          addTerminalLine('output', '', 'USER       PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND')
          addTerminalLine('output', '', 'ros        123  2.1  1.4 123456  7890 pts/0    Sl   10:30   0:05 /opt/ros/...')
        } else {
          addTerminalLine('output', '', `Résultat de: ${command}`)
        }
      }, 500)
    }
    
    const executeLogCommand = (command) => {
      setTimeout(() => {
        if (command.includes('grep ERROR')) {
          const errorLogs = rosLogs.value.filter(log => log.level === 'error')
          errorLogs.forEach(log => {
            addTerminalLine('error', log.node, log.message)
          })
        } else {
          addTerminalLine('output', '', `Log command: ${command}`)
        }
      }, 300)
    }
    
    const previousCommand = () => {
      if (commandHistoryIndex.value > 0) {
        commandHistoryIndex.value--
        currentCommand.value = commandHistory.value[commandHistoryIndex.value]
      }
    }
    
    const nextCommand = () => {
      if (commandHistoryIndex.value < commandHistory.value.length - 1) {
        commandHistoryIndex.value++
        currentCommand.value = commandHistory.value[commandHistoryIndex.value]
      } else {
        commandHistoryIndex.value = commandHistory.value.length
        currentCommand.value = ''
      }
    }
    
    const clearTerminal = () => {
      terminalLines.value = []
      addTerminalLine('info', 'system', 'Terminal effacé')
    }
    
    const saveTerminalOutput = () => {
      const output = terminalLines.value.map(line => 
        `[${formatTerminalTime(line.timestamp)}] ${line.prefix} ${line.content}`
      ).join('\n')
      
      const blob = new Blob([output], { type: 'text/plain' })
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = `terminal-output-${new Date().toISOString().slice(0, 10)}.txt`
      a.click()
      URL.revokeObjectURL(url)
    }
    
    // Logs
    const addLog = (level, node, message) => {
      const newLog = {
        id: Date.now() + Math.random(),
        timestamp: Date.now(),
        level,
        node,
        message
      }
      rosLogs.value.unshift(newLog)
      
      if (rosLogs.value.length > 500) {
        rosLogs.value = rosLogs.value.slice(0, 500)
      }
    }
    
    const clearLogs = () => {
      rosLogs.value = []
      addTerminalLine('info', 'logs', 'Logs ROS effacés')
    }
    
    const exportLogs = () => {
      const logData = rosLogs.value.map(log => ({
        timestamp: new Date(log.timestamp).toISOString(),
        level: log.level,
        node: log.node,
        message: log.message
      }))
      
      const dataStr = JSON.stringify(logData, null, 2)
      const blob = new Blob([dataStr], { type: 'application/json' })
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = `ros-logs-${new Date().toISOString().slice(0, 10)}.json`
      a.click()
      URL.revokeObjectURL(url)
    }
    
    // Actions rapides
    const emergencyStop = () => {
      if (confirm('ARRÊT D\'URGENCE - Arrêter tous les robots immédiatement ?')) {
        executeROSCommand('ros2 service call /emergency_stop std_srvs/srv/Trigger')
        addTerminalLine('error', 'emergency', 'ARRÊT D\'URGENCE ACTIVÉ')
        addLog('fatal', 'emergency_controller', 'Emergency stop activated by operator')
      }
    }
    
    const robotGoHome = () => {
      executeROSCommand('ros2 service call /robot/go_home std_srvs/srv/Trigger')
      addTerminalLine('info', 'robot', 'Robot retour à la base demandé')
    }
    
    const calibrateAll = () => {
      addTerminalLine('info', 'calibration', 'Calibration complète démarrée...')
      executeROSCommand('ros2 launch tekbot_calibration calibrate_all.launch.py')
    }
    
    const requestManualReset = () => {
      if (confirm('Demander une réinitialisation manuelle (-10 points) ?')) {
        currentScore.value -= 10
        executeROSCommand('ros2 service call /game/request_manual_reset std_srvs/srv/Trigger')
        addTerminalLine('warning', 'game', 'Réinitialisation manuelle demandée (-10 pts)')
      }
    }
    
    const pauseConveyor = () => {
      executeROSCommand('ros2 service call /conveyor/pause std_srvs/srv/Trigger')
      addTerminalLine('info', 'conveyor', 'Pause convoyeur demandée')
    }
    
    const restartArm = () => {
      executeROSCommand('ros2 service call /arm/restart std_srvs/srv/Trigger')
      addTerminalLine('info', 'arm', 'Redémarrage du bras robotique')
    }
    
    // Gestion des nœuds
    const refreshNodes = () => {
      addTerminalLine('info', 'nodes', 'Actualisation de la liste des nœuds...')
      executeROSCommand('ros2 node list')
    }
    
    // Timers et intervalles
    let matchTimer = null
    let metricsTimer = null
    let nodesTimer = null
    
    const startMatchTimer = () => {
      matchTimer = setInterval(() => {
        if (matchRunning.value && matchTimeRemaining.value > 0) {
          matchTimeRemaining.value--
          
          if (matchTimeRemaining.value === 0) {
            matchRunning.value = false
            addTerminalLine('info', 'match', 'TEMPS ÉCOULÉ - Match terminé')
            addLog('info', 'game_manager', 'Match ended - time limit reached')
          }
        }
      }, 1000)
    }
    
    onMounted(() => {
      // Simuler l'initialisation du système
      setTimeout(() => {
        systemReady.value = true
        addTerminalLine('success', 'system', 'Système TEKBOT prêt pour le match')
      }, 2000)
      
      // Métriques système
      metricsTimer = setInterval(() => {
        systemMetrics.value.cpu = Math.floor(Math.random() * 30) + 40
        systemMetrics.value.ram = Math.floor(Math.random() * 20) + 60
        systemMetrics.value.messagesPerSec = Math.floor(Math.random() * 100) + 100
        systemMetrics.value.network = (Math.random() * 50 + 10).toFixed(1)
      }, 5000)
      
      // Auto-refresh des nœuds
      nodesTimer = setInterval(() => {
        if (autoRefreshNodes.value) {
          // Simuler les changements de statut
          activeNodes.value.forEach(node => {
            if (Math.random() > 0.9) {
              node.hz = Math.floor(Math.random() * 20) + 5
            }
          })
        }
      }, 3000)
      
      // Simuler des logs périodiques
      setInterval(() => {
        if (Math.random() > 0.7) {
          const levels = ['info', 'warning', 'debug']
          const nodes = ['robot_node', 'conveyor_node', 'arm_node', 'camera_node']
          const messages = [
            'Heartbeat received',
            'Task completed successfully',
            'Sensor reading updated',
            'Communication check OK'
          ]
          
          addLog(
            levels[Math.floor(Math.random() * levels.length)],
            nodes[Math.floor(Math.random() * nodes.length)],
            messages[Math.floor(Math.random() * messages.length)]
          )
        }
      }, 4000)
    })
    
    onUnmounted(() => {
      if (matchTimer) clearInterval(matchTimer)
      if (metricsTimer) clearInterval(metricsTimer)
      if (nodesTimer) clearInterval(nodesTimer)
    })
    
    return {
      // État
      matchRunning,
      matchTimeRemaining,
      currentScore,
      systemReady,
      rvizConnected,
      rvizMode,
      rvizWebUrl,
      terminalLines,
      currentCommand,
      selectedTerminalMode,
      rosLogs,
      selectedLogLevel,
      activeNodes,
      autoRefreshNodes,
      systemComponents,
      systemMetrics,
      
      // Computed
      systemStatusClass,
      systemStatusText,
      timerClass,
      terminalPrompt,
      terminalPlaceholder,
      filteredLogs,
      
      // Méthodes
      formatTime,
      formatTerminalTime,
      formatLogTime,
      getNodeIcon,
      startMatch,
      pauseMatch,
      stopMatch,
      resetMatch,
      refreshRViz,
      centerRobot,
      toggleRVizMode,
      connectRViz,
      pingComponent,
      pingAllSystems,
      runFullDiagnostic,
      executeCommand,
      previousCommand,
      nextCommand,
      clearTerminal,
      saveTerminalOutput,
      clearLogs,
      exportLogs,
      emergencyStop,
      robotGoHome,
      calibrateAll,
      requestManualReset,
      pauseConveyor,
      restartArm,
      refreshNodes
    }
  }
}
</script>

<style scoped>
.match-supervision-container {
  display: flex;
  flex-direction: column;
  height: 100%;
  gap: var(--spacing-md);
}

/* En-tête de contrôle */
.match-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  background: var(--bg-card);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-lg);
  border: 1px solid var(--border-color);
}

.match-controls {
  display: flex;
  gap: var(--spacing-md);
}

.match-status {
  display: flex;
  align-items: center;
  gap: var(--spacing-lg);
}

.match-timer {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 2rem;
  font-weight: bold;
  color: var(--text-primary);
}

.match-timer.timer-warning {
  color: var(--warning-color);
}

.match-timer.timer-critical {
  color: var(--danger-color);
  animation: pulse 1s infinite;
}

.match-score {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 1.5rem;
  font-weight: bold;
  color: var(--secondary-color);
}

/* Layout principal */
.supervision-layout {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: var(--spacing-lg);
  flex: 1;
  min-height: 0;
}

.left-panel,
.center-panel,
.right-panel {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
  min-height: 0;
}

/* RViz Container */
.rviz-container {
  flex: 2;
}

.rviz-controls {
  display: flex;
  gap: var(--spacing-xs);
}

.rviz-display {
  flex: 1;
  min-height: 300px;
  position: relative;
}

.rviz-frame {
  width: 100%;
  height: 100%;
  border-radius: var(--border-radius-md);
  overflow: hidden;
}

.rviz-placeholder,
.rviz-disconnected {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 100%;
  background: var(--bg-tertiary);
  border-radius: var(--border-radius-md);
  color: var(--text-muted);
}

.placeholder-icon,
.disconnect-icon {
  font-size: 3rem;
  margin-bottom: var(--spacing-md);
}

/* Système de ping */
.ping-system {
  flex: 1;
}

.ping-controls {
  display: flex;
  gap: var(--spacing-xs);
}

.ping-grid {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-xs);
}

.ping-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-sm);
  background: var(--bg-surface);
  border-radius: var(--border-radius-md);
  cursor: pointer;
  transition: all 0.2s;
  border-left: 3px solid transparent;
}

.ping-item:hover {
  background: var(--bg-tertiary);
}

.ping-item.connected {
  border-left-color: var(--secondary-color);
}

.ping-item.warning {
  border-left-color: var(--warning-color);
}

.ping-item.disconnected {
  border-left-color: var(--danger-color);
  opacity: 0.6;
}

.ping-info {
  flex: 1;
}

.ping-name {
  font-weight: 500;
  color: var(--text-primary);
}

.ping-details {
  font-size: 0.75rem;
  color: var(--text-muted);
}

.ping-status {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
}

.ping-latency {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 0.75rem;
  color: var(--text-secondary);
}

.ping-indicator {
  width: 8px;
  height: 8px;
  border-radius: 50%;
}

.ping-indicator.connected {
  background: var(--secondary-color);
}

.ping-indicator.warning {
  background: var(--warning-color);
}

.ping-indicator.disconnected {
  background: var(--danger-color);
}

/* Terminal */
.terminal-container {
  flex: 2;
}

.terminal-controls {
  display: flex;
  gap: var(--spacing-sm);
  align-items: center;
}

.terminal-mode-select {
  background: var(--bg-tertiary);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-sm);
  padding: var(--spacing-xs) var(--spacing-sm);
  color: var(--text-primary);
  font-size: 0.75rem;
}

.terminal-display {
  display: flex;
  flex-direction: column;
  height: 300px;
  background: #1a1a1a;
  border-radius: var(--border-radius-md);
  overflow: hidden;
}

.terminal-output {
  flex: 1;
  overflow-y: auto;
  padding: var(--spacing-sm);
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 0.75rem;
  line-height: 1.4;
}

.terminal-line {
  display: flex;
  gap: var(--spacing-xs);
  margin-bottom: 2px;
}

.terminal-line.command {
  color: #ffffff;
}

.terminal-line.info {
  color: #00ff00;
}

.terminal-line.success {
  color: #00ff00;
}

.terminal-line.warning {
  color: #ffaa00;
}

.terminal-line.error {
  color: #ff4444;
}

.terminal-line.output {
  color: #cccccc;
}

.terminal-timestamp {
  color: #666666;
  min-width: 60px;
}

.terminal-prefix {
  color: #00aaff;
  min-width: 80px;
}

.terminal-content {
  flex: 1;
}

.terminal-input {
  display: flex;
  align-items: center;
  padding: var(--spacing-sm);
  background: #2a2a2a;
  border-top: 1px solid #444444;
}

.terminal-prompt {
  color: #00aaff;
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 0.75rem;
  margin-right: var(--spacing-xs);
}

.terminal-field {
  flex: 1;
  background: transparent;
  border: none;
  color: #ffffff;
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 0.75rem;
  outline: none;
}

.terminal-field::placeholder {
  color: #666666;
}

/* Logs */
.logs-container {
  flex: 1;
}

.logs-controls {
  display: flex;
  gap: var(--spacing-sm);
  align-items: center;
}

.log-level-filter {
  background: var(--bg-tertiary);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-sm);
  padding: var(--spacing-xs) var(--spacing-sm);
  color: var(--text-primary);
  font-size: 0.75rem;
}

.logs-display {
  flex: 1;
  overflow-y: auto;
  max-height: 200px;
}

.log-entry {
  display: grid;
  grid-template-columns: auto auto 100px 1fr;
  gap: var(--spacing-sm);
  padding: var(--spacing-xs) var(--spacing-sm);
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 0.7rem;
  border-bottom: 1px solid var(--border-color);
}

.log-entry.debug {
  color: var(--text-muted);
}

.log-entry.info {
  color: var(--text-secondary);
}

.log-entry.warning {
  color: var(--warning-color);
}

.log-entry.error {
  color: var(--danger-color);
}

.log-timestamp {
  color: var(--text-muted);
}

.log-level {
  font-weight: bold;
  text-transform: uppercase;
}

.log-node {
  color: var(--primary-color);
}

/* Nœuds ROS */
.nodes-monitor {
  flex: 1;
}

.nodes-controls {
  display: flex;
  gap: var(--spacing-sm);
  align-items: center;
}

.auto-refresh-toggle {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-size: 0.75rem;
}

.auto-refresh-toggle input[type="checkbox"] {
  margin: 0;
}

.nodes-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-xs);
}

.node-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-sm);
  background: var(--bg-surface);
  border-radius: var(--border-radius-md);
  border-left: 3px solid transparent;
}

.node-item.connected {
  border-left-color: var(--secondary-color);
}

.node-item.warning {
  border-left-color: var(--warning-color);
}

.node-item.disconnected {
  border-left-color: var(--danger-color);
  opacity: 0.6;
}

.node-info {
  flex: 1;
}

.node-name {
  font-weight: 500;
  font-size: 0.75rem;
  color: var(--text-primary);
}

.node-topics {
  font-size: 0.7rem;
  color: var(--text-muted);
}

.node-metrics {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
}

.node-hz {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 0.7rem;
  color: var(--text-secondary);
}

.node-status-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
}

.node-status-dot.connected {
  background: var(--secondary-color);
}

.node-status-dot.warning {
  background: var(--warning-color);
}

.node-status-dot.disconnected {
  background: var(--danger-color);
}

/* Actions rapides */
.quick-actions {
  flex: 1;
}

.actions-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: var(--spacing-sm);
}

.action-btn {
  padding: var(--spacing-sm);
  background: var(--bg-surface);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-md);
  color: var(--text-primary);
  font-size: 0.7rem;
  cursor: pointer;
  transition: all 0.2s;
}

.action-btn:hover {
  background: var(--bg-tertiary);
  border-color: var(--primary-color);
}

.action-btn:first-child {
  background: rgba(239, 68, 68, 0.2);
  border-color: var(--danger-color);
  color: var(--danger-color);
}

/* Métriques système */
.system-metrics {
  flex: 1;
}

.metrics-display {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.metric-item {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-xs);
}

.metric-label {
  font-size: 0.75rem;
  color: var(--text-muted);
}

.metric-value {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--text-primary);
}

.metric-bar {
  height: 4px;
  background: var(--bg-tertiary);
  border-radius: 2px;
  overflow: hidden;
}

.metric-fill {
  height: 100%;
  background: linear-gradient(90deg, var(--secondary-color), var(--primary-color));
  transition: width 0.3s ease;
}

/* Animations */
@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

@keyframes spin {
  from { transform: rotate(0deg); }
  to { transform: rotate(360deg); }
}

.animate-pulse {
  animation: pulse 2s infinite;
}
</style>