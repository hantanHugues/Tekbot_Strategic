<template>
  <div class="system-health-container">
    <div class="health-header">
      <h2>État du Système</h2>
      <p>Surveillance des composants et diagnostic en temps réel</p>
    </div>
    
    <div class="health-grid">
      <!-- État Global -->
      <div class="card system-overview">
        <div class="card-header">
          <h3 class="card-title">État Global du Système</h3>
          <div class="system-status" :class="globalSystemStatus.class">
            <div class="status-icon">{{ globalSystemStatus.icon }}</div>
            <span>{{ globalSystemStatus.text }}</span>
          </div>
        </div>
        
        <div class="system-metrics">
          <div class="metric-item">
            <div class="metric-label">Uptime:</div>
            <div class="metric-value">{{ systemUptime }}</div>
          </div>
          <div class="metric-item">
            <div class="metric-label">Nœuds Actifs:</div>
            <div class="metric-value">{{ activeNodes }}/{{ totalNodes }}</div>
          </div>
          <div class="metric-item">
            <div class="metric-label">Messages/sec:</div>
            <div class="metric-value">{{ messageRate }}</div>
          </div>
        </div>
        
        <div class="system-actions">
          <button class="btn btn-primary" @click="refreshSystemStatus">
            🔄 Actualiser
          </button>
          <button class="btn btn-secondary" @click="exportDiagnostics">
            📄 Exporter Diagnostics
          </button>
        </div>
      </div>
      
      <!-- État des Nœuds ROS2 -->
      <div class="card ros-nodes">
        <div class="card-header">
          <h3 class="card-title">Nœuds ROS2</h3>
          <div class="refresh-indicator" :class="{ active: refreshing }">
            🔄
          </div>
        </div>
        
        <div class="nodes-list">
          <div v-for="node in rosNodes" :key="node.name" 
               class="node-item" :class="node.status">
            <div class="node-info">
              <div class="node-name">
                <span class="node-icon">{{ getNodeIcon(node.type) }}</span>
                {{ node.name }}
              </div>
              <div class="node-description">{{ node.description }}</div>
            </div>
            
            <div class="node-metrics">
              <div class="node-metric">
                <span class="metric-label">Heartbeat:</span>
                <span class="metric-value">{{ node.lastHeartbeat }}ms</span>
              </div>
              <div class="node-metric">
                <span class="metric-label">Messages:</span>
                <span class="metric-value">{{ node.messageCount }}</span>
              </div>
            </div>
            
            <div class="node-status-indicator" :class="getNodeStatusClass(node.status)">
              <div class="status-dot animate-pulse" v-if="node.status === 'connected'"></div>
              <div class="status-dot" v-else></div>
              <span>{{ getNodeStatusText(node.status) }}</span>
            </div>
          </div>
        </div>
      </div>
    </div>
    
    <!-- Diagnostics Détaillés -->
    <div class="diagnostics-section">
      <div class="card hardware-diagnostics">
        <div class="card-header">
          <h3 class="card-title">Diagnostics Matériel</h3>
          <div class="diagnostic-controls">
            <button class="btn btn-outline btn-sm" @click="runDiagnostics">
              ⚙️ Lancer Diagnostics
            </button>
            <button class="btn btn-outline btn-sm" @click="clearDiagnostics">
              🗑️ Effacer
            </button>
          </div>
        </div>
        
        <div class="diagnostics-grid">
          <!-- Robot Mobile -->
          <div class="diagnostic-group">
            <h4>🤖 Robot Mobile (Robomaster X3)</h4>
            <div class="diagnostic-items">
              <div class="diagnostic-item" :class="robotDiagnostics.battery.status">
                <span class="diagnostic-label">Batterie:</span>
                <span class="diagnostic-value">{{ robotDiagnostics.battery.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
              <div class="diagnostic-item" :class="robotDiagnostics.motors.status">
                <span class="diagnostic-label">Moteurs:</span>
                <span class="diagnostic-value">{{ robotDiagnostics.motors.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
              <div class="diagnostic-item" :class="robotDiagnostics.sensors.status">
                <span class="diagnostic-label">Capteurs:</span>
                <span class="diagnostic-value">{{ robotDiagnostics.sensors.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
              <div class="diagnostic-item" :class="robotDiagnostics.navigation.status">
                <span class="diagnostic-label">Navigation:</span>
                <span class="diagnostic-value">{{ robotDiagnostics.navigation.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
            </div>
          </div>
          
          <!-- Station de Tri -->
          <div class="diagnostic-group">
            <h4>🏢 Station de Tri</h4>
            <div class="diagnostic-items">
              <div class="diagnostic-item" :class="conveyorDiagnostics.motor.status">
                <span class="diagnostic-label">Moteur Convoyeur:</span>
                <span class="diagnostic-value">{{ conveyorDiagnostics.motor.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
              <div class="diagnostic-item" :class="conveyorDiagnostics.ldrSensors.status">
                <span class="diagnostic-label">Capteurs LDR:</span>
                <span class="diagnostic-value">{{ conveyorDiagnostics.ldrSensors.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
              <div class="diagnostic-item" :class="conveyorDiagnostics.colorSensor.status">
                <span class="diagnostic-label">Capteur Couleur:</span>
                <span class="diagnostic-value">{{ conveyorDiagnostics.colorSensor.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
              <div class="diagnostic-item" :class="armDiagnostics.servos.status">
                <span class="diagnostic-label">Bras Robotique:</span>
                <span class="diagnostic-value">{{ armDiagnostics.servos.value }}</span>
                <div class="diagnostic-indicator"></div>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      <!-- Logs Système -->
      <div class="card system-logs">
        <div class="card-header">
          <h3 class="card-title">Logs Système</h3>
          <div class="log-controls">
            <select v-model="selectedLogLevel" class="log-filter">
              <option value="all">Tous</option>
              <option value="error">Erreurs</option>
              <option value="warning">Warnings</option>
              <option value="info">Info</option>
            </select>
            <button class="btn btn-outline btn-sm" @click="clearLogs">
              🗑️ Effacer
            </button>
          </div>
        </div>
        
        <div class="logs-container">
          <div v-for="log in filteredLogs" :key="log.id" 
               class="log-entry" :class="log.level">
            <div class="log-timestamp">{{ formatTimestamp(log.timestamp) }}</div>
            <div class="log-level">{{ log.level.toUpperCase() }}</div>
            <div class="log-source">{{ log.source }}</div>
            <div class="log-message">{{ log.message }}</div>
          </div>
          
          <div v-if="filteredLogs.length === 0" class="empty-logs">
            <div class="empty-icon">📜</div>
            <p>Aucun log disponible</p>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import ROSService from '../services/ROSService.js'

export default {
  name: 'SystemHealth',
  setup() {
    const refreshing = ref(false)
    const selectedLogLevel = ref('all')
    
    const systemUptime = ref('02:15:43')
    const activeNodes = ref(4)
    const totalNodes = ref(5)
    const messageRate = ref(127)
    
    const rosNodes = ref([
      {
        name: '/robot_node',
        type: 'robot',
        description: 'Contrôle du robot mobile Robomaster X3',
        status: 'connected',
        lastHeartbeat: 45,
        messageCount: 1247
      },
      {
        name: '/conveyor_node',
        type: 'conveyor',
        description: 'Gestion du convoyeur et capteurs',
        status: 'connected',
        lastHeartbeat: 32,
        messageCount: 892
      },
      {
        name: '/arm_node',
        type: 'arm',
        description: 'Contrôle du bras robotique Dofbot',
        status: 'warning',
        lastHeartbeat: 156,
        messageCount: 234
      },
      {
        name: '/game_manager',
        type: 'system',
        description: 'Gestionnaire de match et scoring',
        status: 'connected',
        lastHeartbeat: 23,
        messageCount: 456
      },
      {
        name: '/camera_node',
        type: 'sensor',
        description: 'Traitement d\'image et QR codes',
        status: 'disconnected',
        lastHeartbeat: 0,
        messageCount: 0
      }
    ])
    
    const systemLogs = ref([
      {
        id: 1,
        timestamp: Date.now() - 30000,
        level: 'info',
        source: 'robot_node',
        message: 'Navigation vers quartier Q-Commercial-01 démarrée'
      },
      {
        id: 2,
        timestamp: Date.now() - 25000,
        level: 'warning',
        source: 'arm_node',
        message: 'Calibration des servos requise - précision réduite'
      },
      {
        id: 3,
        timestamp: Date.now() - 15000,
        level: 'error',
        source: 'camera_node',
        message: 'Perte de connexion avec le module caméra'
      },
      {
        id: 4,
        timestamp: Date.now() - 10000,
        level: 'info',
        source: 'conveyor_node',
        message: 'Cube orange détecté et trié avec succès'
      },
      {
        id: 5,
        timestamp: Date.now() - 5000,
        level: 'warning',
        source: 'robot_node',
        message: 'Niveau de batterie faible (18% restant)'
      }
    ])
    
    // Diagnostics matériel
    const robotDiagnostics = ref({
      battery: { status: 'warning', value: '18% (3.2V)' },
      motors: { status: 'ok', value: 'Normaux' },
      sensors: { status: 'ok', value: 'Opérationnels' },
      navigation: { status: 'ok', value: 'Actif' }
    })
    
    const conveyorDiagnostics = ref({
      motor: { status: 'ok', value: 'Tournant (1200 RPM)' },
      ldrSensors: { status: 'ok', value: 'Calibrés' },
      colorSensor: { status: 'ok', value: 'RGB actif' }
    })
    
    const armDiagnostics = ref({
      servos: { status: 'warning', value: 'Calibration nécessaire' }
    })
    
    // Computed properties
    const globalSystemStatus = computed(() => {
      const errorNodes = rosNodes.value.filter(n => n.status === 'disconnected').length
      const warningNodes = rosNodes.value.filter(n => n.status === 'warning').length
      
      if (errorNodes > 0) {
        return { 
          class: 'status-error', 
          icon: '❌', 
          text: `${errorNodes} composant(s) hors ligne` 
        }
      }
      
      if (warningNodes > 0) {
        return { 
          class: 'status-warning', 
          icon: '⚠️', 
          text: `${warningNodes} alerte(s) système` 
        }
      }
      
      return { 
        class: 'status-ok', 
        icon: '✅', 
        text: 'Tous les systèmes opérationnels' 
      }
    })
    
    const filteredLogs = computed(() => {
      if (selectedLogLevel.value === 'all') {
        return systemLogs.value
      }
      return systemLogs.value.filter(log => log.level === selectedLogLevel.value)
    })
    
    // Méthodes
    const getNodeIcon = (type) => {
      const icons = {
        robot: '🤖',
        conveyor: '🏢',
        arm: '🥾',
        system: '⚙️',
        sensor: '📷'
      }
      return icons[type] || '💻'
    }
    
    const getNodeStatusClass = (status) => {
      const classes = {
        connected: 'status-connected',
        warning: 'status-warning',
        disconnected: 'status-disconnected'
      }
      return classes[status] || 'status-disconnected'
    }
    
    const getNodeStatusText = (status) => {
      const texts = {
        connected: 'Connecté',
        warning: 'Alerte',
        disconnected: 'Déconnecté'
      }
      return texts[status] || 'Inconnu'
    }
    
    const formatTimestamp = (timestamp) => {
      const date = new Date(timestamp)
      return date.toLocaleTimeString('fr-FR', {
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
      })
    }
    
    const refreshSystemStatus = async () => {
      refreshing.value = true
      
      // Simuler la vérification des nœuds
      setTimeout(() => {
        rosNodes.value.forEach(node => {
          if (Math.random() > 0.1) {
            node.lastHeartbeat = Math.floor(Math.random() * 100) + 10
            node.status = node.lastHeartbeat > 100 ? 'warning' : 'connected'
          }
        })
        
        activeNodes.value = rosNodes.value.filter(n => n.status === 'connected').length
        messageRate.value = Math.floor(Math.random() * 200) + 50
        
        refreshing.value = false
      }, 2000)
    }
    
    const exportDiagnostics = () => {
      const diagnosticData = {
        timestamp: new Date().toISOString(),
        systemUptime: systemUptime.value,
        nodes: rosNodes.value,
        robotDiagnostics: robotDiagnostics.value,
        conveyorDiagnostics: conveyorDiagnostics.value,
        armDiagnostics: armDiagnostics.value,
        recentLogs: systemLogs.value.slice(0, 20)
      }
      
      const dataStr = JSON.stringify(diagnosticData, null, 2)
      const blob = new Blob([dataStr], { type: 'application/json' })
      const url = URL.createObjectURL(blob)
      
      const a = document.createElement('a')
      a.href = url
      a.download = `tekbot-diagnostics-${new Date().toISOString().slice(0, 10)}.json`
      a.click()
      
      URL.revokeObjectURL(url)
    }
    
    const runDiagnostics = async () => {
      // Simuler l'exécution de tests de diagnostic
      const tests = ['Batterie', 'Moteurs', 'Capteurs', 'Communication']
      
      for (const test of tests) {
        addLog('info', 'diagnostic', `Test ${test} en cours...`)
        await new Promise(resolve => setTimeout(resolve, 1000))
        
        if (Math.random() > 0.2) {
          addLog('info', 'diagnostic', `Test ${test}: PASS`)
        } else {
          addLog('warning', 'diagnostic', `Test ${test}: Attention requis`)
        }
      }
      
      addLog('info', 'diagnostic', 'Diagnostic complet terminé')
    }
    
    const addLog = (level, source, message) => {
      const newLog = {
        id: Date.now() + Math.random(),
        timestamp: Date.now(),
        level,
        source,
        message
      }
      
      systemLogs.value.unshift(newLog)
      
      // Limiter à 100 logs
      if (systemLogs.value.length > 100) {
        systemLogs.value = systemLogs.value.slice(0, 100)
      }
    }
    
    const clearDiagnostics = () => {
      if (confirm('Effacer tous les résultats de diagnostic ?')) {
        systemLogs.value = systemLogs.value.filter(log => log.source !== 'diagnostic')
      }
    }
    
    const clearLogs = () => {
      if (confirm('Effacer tous les logs système ?')) {
        systemLogs.value = []
      }
    }
    
    // Simulation de l'uptime
    let uptimeInterval = null
    let logInterval = null
    
    onMounted(() => {
      // Simuler l'uptime
      let seconds = 8143 // 2h 15m 43s
      uptimeInterval = setInterval(() => {
        seconds++
        const hours = Math.floor(seconds / 3600)
        const minutes = Math.floor((seconds % 3600) / 60)
        const secs = seconds % 60
        systemUptime.value = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`
      }, 1000)
      
      // Simuler des logs périodiques
      logInterval = setInterval(() => {
        if (Math.random() > 0.7) {
          const sources = ['robot_node', 'conveyor_node', 'arm_node', 'game_manager']
          const levels = ['info', 'warning']
          const messages = [
            'Heartbeat reçu',
            'Opération complétée',
            'Mise à jour de statut',
            'Vérification périodique OK'
          ]
          
          addLog(
            levels[Math.floor(Math.random() * levels.length)],
            sources[Math.floor(Math.random() * sources.length)],
            messages[Math.floor(Math.random() * messages.length)]
          )
        }
      }, 5000)
    })
    
    onUnmounted(() => {
      if (uptimeInterval) clearInterval(uptimeInterval)
      if (logInterval) clearInterval(logInterval)
    })
    
    return {
      refreshing,
      selectedLogLevel,
      systemUptime,
      activeNodes,
      totalNodes,
      messageRate,
      rosNodes,
      robotDiagnostics,
      conveyorDiagnostics,
      armDiagnostics,
      globalSystemStatus,
      filteredLogs,
      getNodeIcon,
      getNodeStatusClass,
      getNodeStatusText,
      formatTimestamp,
      refreshSystemStatus,
      exportDiagnostics,
      runDiagnostics,
      clearDiagnostics,
      clearLogs
    }
  }
}
</script>

<style scoped>
.system-health-container {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
  height: 100%;
}

.health-header {
  text-align: center;
  margin-bottom: var(--spacing-lg);
}

.health-header h2 {
  color: var(--text-primary);
  margin-bottom: var(--spacing-sm);
}

.health-header p {
  color: var(--text-secondary);
}

.health-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: var(--spacing-lg);
  margin-bottom: var(--spacing-lg);
}

/* État Global */
.system-overview {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
}

.system-status {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-sm) var(--spacing-md);
  border-radius: var(--border-radius-md);
  font-weight: 500;
}

.system-status.status-ok {
  background: rgba(16, 185, 129, 0.2);
  color: var(--secondary-color);
}

.system-status.status-warning {
  background: rgba(245, 158, 11, 0.2);
  color: var(--warning-color);
}

.system-status.status-error {
  background: rgba(239, 68, 68, 0.2);
  color: var(--danger-color);
}

.status-icon {
  font-size: 1.25rem;
}

.system-metrics {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
  background: var(--bg-surface);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-md);
}

.metric-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.metric-label {
  color: var(--text-secondary);
  font-weight: 500;
}

.metric-value {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--text-primary);
}

.system-actions {
  display: flex;
  gap: var(--spacing-md);
}

/* Nœuds ROS2 */
.ros-nodes {
  display: flex;
  flex-direction: column;
}

.refresh-indicator {
  font-size: 1rem;
  color: var(--text-muted);
  transition: color 0.3s;
}

.refresh-indicator.active {
  color: var(--primary-color);
  animation: spin 1s linear infinite;
}

.nodes-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
  flex: 1;
  overflow-y: auto;
  max-height: 400px;
}

.node-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-md);
  background: var(--bg-surface);
  border-radius: var(--border-radius-md);
  border-left: 3px solid transparent;
  transition: all 0.2s;
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
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-weight: 500;
  color: var(--text-primary);
  margin-bottom: 2px;
}

.node-icon {
  font-size: 1rem;
}

.node-description {
  font-size: 0.75rem;
  color: var(--text-muted);
}

.node-metrics {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.node-metric {
  display: flex;
  justify-content: space-between;
  font-size: 0.75rem;
  min-width: 120px;
}

.node-metric .metric-label {
  color: var(--text-muted);
}

.node-metric .metric-value {
  font-family: 'Monaco', 'Menlo', monospace;
  color: var(--text-primary);
}

.node-status-indicator {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-size: 0.75rem;
  font-weight: 500;
}

/* Diagnostics */
.diagnostics-section {
  display: grid;
  grid-template-columns: 2fr 1fr;
  gap: var(--spacing-lg);
}

.hardware-diagnostics {
  display: flex;
  flex-direction: column;
}

.diagnostic-controls {
  display: flex;
  gap: var(--spacing-xs);
}

.diagnostics-grid {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
  flex: 1;
}

.diagnostic-group {
  background: var(--bg-surface);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-md);
}

.diagnostic-group h4 {
  color: var(--text-secondary);
  margin-bottom: var(--spacing-md);
  font-size: 0.875rem;
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
}

.diagnostic-items {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.diagnostic-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-sm);
  background: var(--bg-tertiary);
  border-radius: var(--border-radius-sm);
}

.diagnostic-label {
  font-size: 0.875rem;
  color: var(--text-secondary);
  min-width: 120px;
}

.diagnostic-value {
  flex: 1;
  font-size: 0.875rem;
  color: var(--text-primary);
  font-family: 'Monaco', 'Menlo', monospace;
}

.diagnostic-indicator {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  flex-shrink: 0;
}

.diagnostic-item.ok .diagnostic-indicator {
  background: var(--secondary-color);
}

.diagnostic-item.warning .diagnostic-indicator {
  background: var(--warning-color);
}

.diagnostic-item.error .diagnostic-indicator {
  background: var(--danger-color);
}

/* Logs Système */
.system-logs {
  display: flex;
  flex-direction: column;
}

.log-controls {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
}

.log-filter {
  padding: var(--spacing-xs) var(--spacing-sm);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-sm);
  background: var(--bg-tertiary);
  color: var(--text-primary);
  font-size: 0.75rem;
}

.logs-container {
  flex: 1;
  overflow-y: auto;
  max-height: 500px;
  background: var(--bg-tertiary);
  border-radius: var(--border-radius-md);
  padding: var(--spacing-sm);
}

.log-entry {
  display: grid;
  grid-template-columns: 80px 60px 120px 1fr;
  gap: var(--spacing-sm);
  padding: var(--spacing-xs) var(--spacing-sm);
  font-size: 0.75rem;
  border-radius: var(--border-radius-sm);
  margin-bottom: 2px;
}

.log-entry:hover {
  background: var(--bg-surface);
}

.log-entry.error {
  background: rgba(239, 68, 68, 0.1);
  border-left: 2px solid var(--danger-color);
}

.log-entry.warning {
  background: rgba(245, 158, 11, 0.1);
  border-left: 2px solid var(--warning-color);
}

.log-entry.info {
  border-left: 2px solid var(--primary-color);
}

.log-timestamp {
  font-family: 'Monaco', 'Menlo', monospace;
  color: var(--text-muted);
}

.log-level {
  font-weight: 600;
  text-transform: uppercase;
}

.log-level:contains('ERROR') {
  color: var(--danger-color);
}

.log-level:contains('WARNING') {
  color: var(--warning-color);
}

.log-level:contains('INFO') {
  color: var(--primary-color);
}

.log-source {
  color: var(--text-secondary);
  font-weight: 500;
}

.log-message {
  color: var(--text-primary);
}

.empty-logs {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: var(--spacing-2xl);
  color: var(--text-muted);
}

.empty-icon {
  font-size: 2rem;
  margin-bottom: var(--spacing-md);
}
</style>