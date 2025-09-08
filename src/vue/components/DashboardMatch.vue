<template>
  <div class="dashboard-container">
    <!-- Zone Supérieure - Chronomètre et Score -->
    <div class="top-section">
      <div class="timer-card">
        <div class="card-header">
          <h3 class="card-title">Temps de Match</h3>
          <div class="match-phase" :class="matchPhaseClass">{{ matchPhase }}</div>
        </div>
        <div class="timer-display" :class="timerClass">
          {{ formatTime(matchTime) }}
        </div>
        <div class="timer-progress">
          <div class="progress-bar" :style="{ width: timeProgressPercent + '%' }"></div>
        </div>
      </div>

      <div class="score-card">
        <div class="card-header">
          <h3 class="card-title">Score Live</h3>
          <button class="btn btn-outline btn-sm" @click="showScoreBreakdown = !showScoreBreakdown">
            {{ showScoreBreakdown ? 'Masquer' : 'Détails' }}
          </button>
        </div>
        <div class="score-display">{{ currentScore }}</div>
        <div class="score-trend" :class="scoreTrendClass">
          {{ scoreTrend > 0 ? '+' : '' }}{{ scoreTrend }}
        </div>
      </div>

      <div class="actions-card">
        <div class="card-header">
          <h3 class="card-title">Actions Rapides</h3>
        </div>
        <div class="action-buttons">
          <button class="btn btn-danger" @click="requestManualReset" :disabled="!rosConnected">
            🚨 Demander Reset
          </button>
          <button class="btn btn-secondary" @click="addPenalty">
            ⚠️ Ajouter Pénalité
          </button>
        </div>
      </div>
    </div>

    <!-- Zone Principale - 3 colonnes -->
    <div class="main-section">
      <!-- Colonne Gauche - Carte et Stratégie -->
      <div class="left-column">
        <div class="card map-card">
          <div class="card-header">
            <h3 class="card-title">Carte de l'Arène</h3>
            <div class="map-controls">
              <button class="btn btn-outline btn-sm">🔄 Actualiser</button>
              <button class="btn btn-outline btn-sm">🎯 Centrer Robot</button>
            </div>
          </div>
          <div class="map-container">
            <div class="arena-map">
              <!-- Robot position -->
              <div 
                class="robot-marker" 
                :style="{ 
                  left: robotPosition.x + '%', 
                  top: robotPosition.y + '%',
                  transform: `rotate(${robotPosition.orientation}deg)`
                }"
              >
                🤖
              </div>
              
              <!-- Quartiers scannés -->
              <div 
                v-for="quartier in scannedQuartiers" 
                :key="quartier.id"
                class="quartier-marker"
                :style="{ left: quartier.x + '%', top: quartier.y + '%' }"
                :class="{ completed: quartier.completed }"
              >
                <div class="quartier-icon">{{ getQuartierIcon(quartier.type) }}</div>
                <div class="quartier-label">{{ quartier.name }}</div>
              </div>
            </div>
          </div>
        </div>

        <div class="card mission-status">
          <div class="card-header">
            <h3 class="card-title">Statut des Missions</h3>
          </div>
          <div class="mission-list">
            <div v-for="mission in missions" :key="mission.id" class="mission-item" :class="mission.status">
              <div class="mission-icon">{{ getMissionIcon(mission.status) }}</div>
              <div class="mission-info">
                <div class="mission-name">{{ mission.name }}</div>
                <div class="mission-progress">{{ mission.progress }}</div>
              </div>
              <div class="mission-points">+{{ mission.points }}pts</div>
            </div>
          </div>
        </div>
      </div>

      <!-- Colonne Centrale - Journal des Événements -->
      <div class="center-column">
        <div class="card event-log">
          <div class="card-header">
            <h3 class="card-title">Journal des Événements</h3>
            <div class="log-controls">
              <button class="btn btn-outline btn-sm" @click="clearLog">🗑️ Effacer</button>
              <button class="btn btn-outline btn-sm" @click="exportLog">💾 Exporter</button>
            </div>
          </div>
          <div class="log-container">
            <div 
              v-for="event in gameEvents" 
              :key="event.id"
              class="log-entry"
              :class="event.type"
            >
              <div class="log-time">{{ formatLogTime(event.timestamp) }}</div>
              <div class="log-icon">{{ getEventIcon(event.type) }}</div>
              <div class="log-message">{{ event.message }}</div>
              <div class="log-score" v-if="event.scoreChange">
                {{ event.scoreChange > 0 ? '+' : '' }}{{ event.scoreChange }}
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- Colonne Droite - Supervision Matérielle -->
      <div class="right-column">
        <div class="card robot-status">
          <div class="card-header">
            <h3 class="card-title">Robot Mobile</h3>
            <div class="status-indicator" :class="robotStatusClass">
              <div class="status-dot"></div>
              {{ robotStatusText }}
            </div>
          </div>
          <div class="status-grid">
            <div class="status-item">
              <div class="status-label">Batterie</div>
              <div class="battery-indicator">
                <div class="battery-level" :style="{ width: batteryLevel + '%' }" :class="batteryClass"></div>
                <span class="battery-text">{{ batteryLevel }}%</span>
              </div>
            </div>
            <div class="status-item">
              <div class="status-label">Mode Actuel</div>
              <div class="status-value">{{ robotMode }}</div>
            </div>
            <div class="status-item">
              <div class="status-label">Inventaire</div>
              <div class="inventory-display">
                <div v-for="cube in robotInventory" :key="cube.id" class="cube-item" :class="cube.type">
                  {{ getCubeIcon(cube.type) }}
                </div>
              </div>
            </div>
          </div>
        </div>

        <div class="card station-status">
          <div class="card-header">
            <h3 class="card-title">Station de Tri</h3>
            <div class="status-indicator" :class="conveyorStatusClass">
              <div class="status-dot"></div>
              {{ conveyorStatusText }}
            </div>
          </div>
          <div class="conveyor-info">
            <div class="sensor-status">
              <div class="sensor-item">
                <span class="sensor-label">Couleur Détectée:</span>
                <span class="color-display" :style="{ backgroundColor: detectedColor }">
                  {{ detectedColorName }}
                </span>
              </div>
              <div class="sensor-item">
                <span class="sensor-label">Bras Robotique:</span>
                <span class="arm-status" :class="armStatusClass">{{ armStatusText }}</span>
              </div>
            </div>
            <div class="sort-counter">
              <h4>Cubes Triés</h4>
              <div class="counter-grid">
                <div class="counter-item" v-for="(count, type) in sortedCubes" :key="type">
                  <div class="cube-icon" :class="type">{{ getCubeIcon(type) }}</div>
                  <div class="cube-count">{{ count }}</div>
                </div>
              </div>
            </div>
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
  name: 'DashboardMatch',
  setup() {
    // État du match
    const matchTime = ref(300) // 5 minutes en secondes
    const currentScore = ref(0)
    const previousScore = ref(0)
    const showScoreBreakdown = ref(false)
    const rosConnected = ref(false)

    // Position du robot
    const robotPosition = ref({ x: 50, y: 50, orientation: 0 })
    const robotMode = ref('Navigation')
    const batteryLevel = ref(85)
    const robotInventory = ref([
      { id: 1, type: 'menager' },
      { id: 2, type: 'recyclable' }
    ])

    // État de la station
    const conveyorRunning = ref(true)
    const detectedColor = ref('#ff6b35')
    const detectedColorName = ref('Orange')
    const armStatusText = ref('Inactif')
    const sortedCubes = ref({
      menager: 12,
      recyclable: 8,
      compostable: 5
    })

    // Journal des événements
    const gameEvents = ref([
      {
        id: 1,
        timestamp: Date.now() - 30000,
        type: 'success',
        message: 'Cube Ménager trié avec succès',
        scoreChange: 3
      },
      {
        id: 2,
        timestamp: Date.now() - 15000,
        type: 'info',
        message: 'Robot arrivé au Quartier Commercial',
        scoreChange: null
      },
      {
        id: 3,
        timestamp: Date.now() - 5000,
        type: 'warning',
        message: 'Niveau de batterie faible (15%)',
        scoreChange: null
      }
    ])

    // Quartiers scannés
    const scannedQuartiers = ref([
      { id: 1, name: 'Q1', type: 'commercial', x: 20, y: 30, completed: true },
      { id: 2, name: 'Q2', type: 'residential', x: 70, y: 25, completed: false },
      { id: 3, name: 'Q3', type: 'industrial', x: 60, y: 70, completed: false }
    ])

    // Missions
    const missions = ref([
      { id: 1, name: 'Zone Commerciale', status: 'completed', progress: '100%', points: 15 },
      { id: 2, name: 'Zone Résidentielle', status: 'active', progress: '60%', points: 12 },
      { id: 3, name: 'Parking Final', status: 'pending', progress: '0%', points: 10 }
    ])

    // Computed properties
    const matchPhase = computed(() => {
      if (matchTime.value > 240) return 'Début'
      if (matchTime.value > 120) return 'Milieu'
      if (matchTime.value > 60) return 'Fin'
      return 'Critique'
    })

    const matchPhaseClass = computed(() => {
      const phase = matchPhase.value.toLowerCase()
      return `phase-${phase}`
    })

    const timerClass = computed(() => {
      if (matchTime.value <= 30) return 'timer-critical'
      if (matchTime.value <= 60) return 'timer-warning'
      return ''
    })

    const timeProgressPercent = computed(() => {
      return ((300 - matchTime.value) / 300) * 100
    })

    const scoreTrend = computed(() => currentScore.value - previousScore.value)
    const scoreTrendClass = computed(() => {
      if (scoreTrend.value > 0) return 'trend-positive'
      if (scoreTrend.value < 0) return 'trend-negative'
      return 'trend-neutral'
    })

    const robotStatusClass = computed(() => {
      if (batteryLevel.value < 20) return 'status-danger'
      if (batteryLevel.value < 50) return 'status-warning'
      return 'status-connected'
    })

    const robotStatusText = computed(() => {
      if (batteryLevel.value < 20) return 'Batterie Critique'
      if (batteryLevel.value < 50) return 'Batterie Faible'
      return 'Opérationnel'
    })

    const batteryClass = computed(() => {
      if (batteryLevel.value < 20) return 'battery-critical'
      if (batteryLevel.value < 50) return 'battery-warning'
      return 'battery-good'
    })

    const conveyorStatusClass = computed(() => {
      return conveyorRunning.value ? 'status-connected' : 'status-disconnected'
    })

    const conveyorStatusText = computed(() => {
      return conveyorRunning.value ? 'En Marche' : 'Arrêté'
    })

    const armStatusClass = computed(() => {
      if (armStatusText.value === 'En mouvement') return 'status-warning'
      if (armStatusText.value === 'Inactif') return 'status-disconnected'
      return 'status-connected'
    })

    // Méthodes utilitaires
    const formatTime = (seconds) => {
      const mins = Math.floor(seconds / 60)
      const secs = seconds % 60
      return `${mins}:${secs.toString().padStart(2, '0')}`
    }

    const formatLogTime = (timestamp) => {
      const date = new Date(timestamp)
      return date.toLocaleTimeString('fr-FR', { 
        hour: '2-digit', 
        minute: '2-digit', 
        second: '2-digit' 
      })
    }

    const getQuartierIcon = (type) => {
      const icons = {
        commercial: '🏪',
        residential: '🏠',
        industrial: '🏭'
      }
      return icons[type] || '📍'
    }

    const getMissionIcon = (status) => {
      const icons = {
        completed: '✅',
        active: '🔄',
        pending: '⏳'
      }
      return icons[status] || '❓'
    }

    const getEventIcon = (type) => {
      const icons = {
        success: '✅',
        warning: '⚠️',
        error: '❌',
        info: 'ℹ️'
      }
      return icons[type] || 'ℹ️'
    }

    const getCubeIcon = (type) => {
      const icons = {
        menager: '🗑️',
        recyclable: '♻️',
        compostable: '🌱'
      }
      return icons[type] || '📦'
    }

    // Actions
    const requestManualReset = async () => {
      if (confirm('Demander une réinitialisation manuelle coûtera 10 points. Continuer ?')) {
        currentScore.value -= 10
        addGameEvent('warning', 'Demande de réinitialisation manuelle (-10 pts)', -10)
        
        if (rosConnected.value) {
          try {
            await ROSService.callService('requestManualReset')
          } catch (error) {
            console.error('Erreur lors de la demande de reset:', error)
          }
        }
      }
    }

    const addPenalty = () => {
      const penalty = prompt('Points de pénalité à soustraire:', '5')
      if (penalty && !isNaN(penalty)) {
        const points = parseInt(penalty)
        currentScore.value -= points
        addGameEvent('warning', `Pénalité manuelle (-${points} pts)`, -points)
      }
    }

    const addGameEvent = (type, message, scoreChange = null) => {
      const newEvent = {
        id: Date.now(),
        timestamp: Date.now(),
        type,
        message,
        scoreChange
      }
      gameEvents.value.unshift(newEvent)
      if (gameEvents.value.length > 50) {
        gameEvents.value = gameEvents.value.slice(0, 50)
      }
    }

    const clearLog = () => {
      if (confirm('Effacer tout le journal des événements ?')) {
        gameEvents.value = []
      }
    }

    const exportLog = () => {
      const logData = gameEvents.value.map(event => ({
        time: formatLogTime(event.timestamp),
        type: event.type,
        message: event.message,
        score: event.scoreChange
      }))
      
      const dataStr = JSON.stringify(logData, null, 2)
      const blob = new Blob([dataStr], { type: 'application/json' })
      const url = URL.createObjectURL(blob)
      
      const a = document.createElement('a')
      a.href = url
      a.download = `match-log-${new Date().toISOString().slice(0, 10)}.json`
      a.click()
      
      URL.revokeObjectURL(url)
    }

    // Timer pour le match
    let matchTimer = null

    onMounted(() => {
      // Simuler le timer de match
      matchTimer = setInterval(() => {
        if (matchTime.value > 0) {
          matchTime.value--
        }
      }, 1000)

      // Tentative de connexion ROS
      ROSService.connect().then(() => {
        rosConnected.value = true
        
        // S'abonner aux topics
        ROSService.subscribe('gameScore', (message) => {
          previousScore.value = currentScore.value
          currentScore.value = message.data
        })

        ROSService.subscribe('gameTimer', (message) => {
          matchTime.value = Math.floor(message.data)
        })

        ROSService.subscribe('robotBattery', (message) => {
          batteryLevel.value = Math.floor(message.percentage * 100)
        })

      }).catch(() => {
        rosConnected.value = false
        console.log('Mode démo - ROS non connecté')
      })
    })

    onUnmounted(() => {
      if (matchTimer) {
        clearInterval(matchTimer)
      }
      ROSService.cleanup()
    })

    return {
      // État
      matchTime,
      currentScore,
      showScoreBreakdown,
      rosConnected,
      robotPosition,
      robotMode,
      batteryLevel,
      robotInventory,
      conveyorRunning,
      detectedColor,
      detectedColorName,
      armStatusText,
      sortedCubes,
      gameEvents,
      scannedQuartiers,
      missions,

      // Computed
      matchPhase,
      matchPhaseClass,
      timerClass,
      timeProgressPercent,
      scoreTrend,
      scoreTrendClass,
      robotStatusClass,
      robotStatusText,
      batteryClass,
      conveyorStatusClass,
      conveyorStatusText,
      armStatusClass,

      // Méthodes
      formatTime,
      formatLogTime,
      getQuartierIcon,
      getMissionIcon,
      getEventIcon,
      getCubeIcon,
      requestManualReset,
      addPenalty,
      clearLog,
      exportLog
    }
  }
}
</script>

<style scoped>
.dashboard-container {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
  height: 100%;
}

/* Section supérieure */
.top-section {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: var(--spacing-lg);
  height: 200px;
}

.timer-card,
.score-card,
.actions-card {
  background: var(--bg-card);
  border-radius: var(--border-radius-lg);
  border: 1px solid var(--border-color);
  padding: var(--spacing-lg);
  position: relative;
  overflow: hidden;
}

.timer-card::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: linear-gradient(135deg, rgba(37, 99, 235, 0.1), rgba(16, 185, 129, 0.05));
  pointer-events: none;
}

.timer-display {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 3rem;
  font-weight: bold;
  text-align: center;
  color: var(--text-primary);
  margin: var(--spacing-md) 0;
  text-shadow: 0 0 20px rgba(37, 99, 235, 0.5);
}

.timer-display.timer-warning {
  color: var(--warning-color);
  text-shadow: 0 0 20px rgba(245, 158, 11, 0.5);
}

.timer-display.timer-critical {
  color: var(--danger-color);
  text-shadow: 0 0 20px rgba(239, 68, 68, 0.5);
  animation: pulse 1s infinite;
}

.timer-progress {
  height: 4px;
  background: var(--bg-tertiary);
  border-radius: 2px;
  overflow: hidden;
  margin-top: var(--spacing-md);
}

.progress-bar {
  height: 100%;
  background: linear-gradient(90deg, var(--primary-color), var(--secondary-color));
  transition: width 1s ease;
}

.score-display {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 3rem;
  font-weight: bold;
  text-align: center;
  color: var(--secondary-color);
  margin: var(--spacing-md) 0;
  text-shadow: 0 0 20px rgba(16, 185, 129, 0.5);
}

.score-trend {
  text-align: center;
  font-weight: 600;
  font-size: 1.125rem;
}

.trend-positive { color: var(--secondary-color); }
.trend-negative { color: var(--danger-color); }
.trend-neutral { color: var(--text-muted); }

.match-phase {
  padding: var(--spacing-xs) var(--spacing-sm);
  border-radius: var(--border-radius-sm);
  font-size: 0.75rem;
  font-weight: 600;
  text-transform: uppercase;
}

.phase-début { background: rgba(16, 185, 129, 0.2); color: var(--secondary-color); }
.phase-milieu { background: rgba(245, 158, 11, 0.2); color: var(--warning-color); }
.phase-fin { background: rgba(239, 68, 68, 0.2); color: var(--danger-color); }
.phase-critique { 
  background: rgba(239, 68, 68, 0.3); 
  color: var(--danger-color);
  animation: pulse 2s infinite;
}

.action-buttons {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
  margin-top: var(--spacing-lg);
}

/* Section principale */
.main-section {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: var(--spacing-lg);
  flex: 1;
  min-height: 0;
}

.left-column,
.center-column,
.right-column {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
  min-height: 0;
}

/* Carte de l'arène */
.map-card {
  flex: 1;
}

.map-container {
  flex: 1;
  position: relative;
  min-height: 300px;
}

.arena-map {
  position: relative;
  width: 100%;
  height: 100%;
  background: linear-gradient(45deg, var(--bg-tertiary) 25%, transparent 25%),
              linear-gradient(-45deg, var(--bg-tertiary) 25%, transparent 25%),
              linear-gradient(45deg, transparent 75%, var(--bg-tertiary) 75%),
              linear-gradient(-45deg, transparent 75%, var(--bg-tertiary) 75%);
  background-size: 20px 20px;
  background-position: 0 0, 0 10px, 10px -10px, -10px 0px;
  border: 2px solid var(--border-color);
  border-radius: var(--border-radius-md);
  overflow: hidden;
}

.robot-marker {
  position: absolute;
  font-size: 1.5rem;
  z-index: 10;
  animation: pulse 2s infinite;
  filter: drop-shadow(0 0 10px rgba(37, 99, 235, 0.7));
}

.quartier-marker {
  position: absolute;
  transform: translate(-50%, -50%);
  text-align: center;
  z-index: 5;
}

.quartier-marker.completed {
  opacity: 0.6;
}

.quartier-icon {
  font-size: 1.2rem;
  margin-bottom: 2px;
}

.quartier-label {
  font-size: 0.7rem;
  background: var(--bg-card);
  padding: 2px 4px;
  border-radius: 4px;
  border: 1px solid var(--border-color);
}

/* Journal des événements */
.event-log {
  flex: 1;
}

.log-container {
  flex: 1;
  overflow-y: auto;
  max-height: 400px;
  padding-right: var(--spacing-sm);
}

.log-entry {
  display: grid;
  grid-template-columns: 60px 30px 1fr 60px;
  gap: var(--spacing-sm);
  align-items: center;
  padding: var(--spacing-sm);
  border-radius: var(--border-radius-md);
  margin-bottom: var(--spacing-xs);
  font-size: 0.875rem;
  transition: background-color 0.2s;
}

.log-entry:hover {
  background: var(--bg-surface);
}

.log-entry.success { border-left: 3px solid var(--secondary-color); }
.log-entry.warning { border-left: 3px solid var(--warning-color); }
.log-entry.error { border-left: 3px solid var(--danger-color); }
.log-entry.info { border-left: 3px solid var(--primary-color); }

.log-time {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 0.75rem;
  color: var(--text-muted);
}

.log-score {
  font-weight: 600;
  font-family: 'Monaco', 'Menlo', monospace;
}

.log-score:not(:empty) {
  color: var(--secondary-color);
}

/* Statut des systèmes */
.robot-status,
.station-status {
  background: var(--bg-card);
  border-radius: var(--border-radius-lg);
  border: 1px solid var(--border-color);
  padding: var(--spacing-lg);
}

.status-grid {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
}

.status-item {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-xs);
}

.status-label {
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--text-muted);
  text-transform: uppercase;
}

.status-value {
  color: var(--text-primary);
  font-weight: 500;
}

.battery-indicator {
  position: relative;
  height: 20px;
  background: var(--bg-tertiary);
  border-radius: 10px;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
}

.battery-level {
  position: absolute;
  left: 0;
  top: 0;
  bottom: 0;
  border-radius: 10px;
  transition: width 0.5s ease;
}

.battery-good { background: var(--secondary-color); }
.battery-warning { background: var(--warning-color); }
.battery-critical { background: var(--danger-color); }

.battery-text {
  position: relative;
  z-index: 2;
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--text-primary);
  text-shadow: 0 0 4px rgba(0, 0, 0, 0.8);
}

.inventory-display {
  display: flex;
  gap: var(--spacing-xs);
}

.cube-item {
  padding: var(--spacing-xs);
  border-radius: var(--border-radius-sm);
  font-size: 0.875rem;
}

.cube-item.menager { background: rgba(239, 68, 68, 0.2); }
.cube-item.recyclable { background: rgba(16, 185, 129, 0.2); }
.cube-item.compostable { background: rgba(245, 158, 11, 0.2); }

/* Station de tri */
.conveyor-info {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
}

.sensor-status {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.sensor-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.sensor-label {
  font-size: 0.875rem;
  color: var(--text-secondary);
}

.color-display {
  padding: var(--spacing-xs) var(--spacing-sm);
  border-radius: var(--border-radius-sm);
  color: white;
  font-weight: 500;
  font-size: 0.75rem;
  text-shadow: 0 0 4px rgba(0, 0, 0, 0.8);
}

.arm-status {
  font-weight: 500;
  font-size: 0.875rem;
}

.sort-counter {
  background: var(--bg-surface);
  padding: var(--spacing-md);
  border-radius: var(--border-radius-md);
}

.sort-counter h4 {
  margin-bottom: var(--spacing-sm);
  font-size: 0.875rem;
  color: var(--text-secondary);
}

.counter-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: var(--spacing-sm);
}

.counter-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: var(--spacing-xs);
}

.cube-icon {
  font-size: 1.25rem;
}

.cube-count {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--text-primary);
}

/* Mission status */
.mission-status {
  height: fit-content;
}

.mission-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.mission-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-sm);
  border-radius: var(--border-radius-md);
  background: var(--bg-surface);
}

.mission-item.completed {
  background: rgba(16, 185, 129, 0.1);
  border: 1px solid rgba(16, 185, 129, 0.3);
}

.mission-item.active {
  background: rgba(245, 158, 11, 0.1);
  border: 1px solid rgba(245, 158, 11, 0.3);
}

.mission-item.pending {
  opacity: 0.6;
}

.mission-icon {
  font-size: 1.25rem;
}

.mission-info {
  flex: 1;
}

.mission-name {
  font-weight: 500;
  color: var(--text-primary);
}

.mission-progress {
  font-size: 0.75rem;
  color: var(--text-muted);
}

.mission-points {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--secondary-color);
}

/* Controls */
.map-controls,
.log-controls {
  display: flex;
  gap: var(--spacing-xs);
}

.btn-sm {
  padding: var(--spacing-xs) var(--spacing-sm);
  font-size: 0.75rem;
}
</style>