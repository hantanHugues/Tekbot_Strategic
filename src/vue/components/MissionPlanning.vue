<template>
  <div class="mission-planning-container">
    <div class="planning-header">
      <h2>Planification de Mission</h2>
      <p>Scannage des quartiers et optimisation de l'itinéraire</p>
    </div>
    
    <div class="planning-grid">
      <!-- Section Scan QR -->
      <div class="card qr-scan-section">
        <div class="card-header">
          <h3 class="card-title">Scan des Quartiers</h3>
          <div class="scan-controls">
            <button class="btn btn-primary" @click="startQRScan" :disabled="scanning">
              {{ scanning ? '🔄 Scan en cours...' : '📱 Démarrer Scan' }}
            </button>
            <button class="btn btn-secondary" @click="clearScannedQuartiers">
              🗑️ Effacer
            </button>
          </div>
        </div>
        
        <div class="scanned-quartiers-table">
          <table class="quartiers-table">
            <thead>
              <tr>
                <th>Nom</th>
                <th>Type</th>
                <th>Déchets</th>
                <th>Quantité</th>
                <th>Points</th>
                <th>Statut</th>
              </tr>
            </thead>
            <tbody>
              <tr v-for="quartier in scannedQuartiers" :key="quartier.id" 
                  :class="{ completed: quartier.completed, selected: quartier.selected }"
                  @click="toggleQuartierSelection(quartier.id)">
                <td class="quartier-name">
                  <span class="quartier-icon">{{ getQuartierIcon(quartier.type) }}</span>
                  {{ quartier.name }}
                </td>
                <td>{{ quartier.type }}</td>
                <td>
                  <div class="waste-types">
                    <span v-for="waste in quartier.wasteTypes" :key="waste" 
                          class="waste-tag" :class="waste">
                      {{ getWasteIcon(waste) }}
                    </span>
                  </div>
                </td>
                <td class="quantity">{{ quartier.quantity }}</td>
                <td class="points">{{ quartier.points }}pts</td>
                <td>
                  <div class="status-indicator" :class="getQuartierStatusClass(quartier)">
                    <div class="status-dot"></div>
                    {{ getQuartierStatusText(quartier) }}
                  </div>
                </td>
              </tr>
            </tbody>
          </table>
          
          <div v-if="scannedQuartiers.length === 0" class="empty-state">
            <div class="empty-icon">🔍</div>
            <p>Aucun quartier scanné</p>
            <small>Démarrez le scan QR pour découvrir les quartiers disponibles</small>
          </div>
        </div>
      </div>
      
      <!-- Section Carte Stratégique -->
      <div class="card strategy-map-section">
        <div class="card-header">
          <h3 class="card-title">Carte Stratégique</h3>
          <div class="map-tools">
            <button class="btn btn-outline btn-sm" @click="optimizeRoute">
              ⚡ Optimiser Route
            </button>
            <button class="btn btn-outline btn-sm" @click="resetRoute">
              🔄 Reset
            </button>
          </div>
        </div>
        
        <div class="strategy-map-container">
          <div class="arena-map-planning">
            <!-- Quartiers sur la carte -->
            <div v-for="(quartier, index) in scannedQuartiers" :key="quartier.id"
                 class="map-quartier" 
                 :class="{ selected: quartier.selected, completed: quartier.completed }"
                 :style="{ left: quartier.x + '%', top: quartier.y + '%' }"
                 @click="toggleQuartierSelection(quartier.id)">
              <div class="quartier-marker">
                <span class="marker-icon">{{ getQuartierIcon(quartier.type) }}</span>
                <span class="marker-label">{{ quartier.name }}</span>
                <div v-if="quartier.routeOrder" class="route-number">{{ quartier.routeOrder }}</div>
              </div>
            </div>
            
            <!-- Lignes de route -->
            <svg class="route-lines" viewBox="0 0 100 100">
              <path v-if="routePath" :d="routePath" stroke="#2563eb" stroke-width="0.5" 
                    fill="none" stroke-dasharray="2,1" opacity="0.8"/>
            </svg>
            
            <!-- Position de départ -->
            <div class="start-position" :style="{ left: '10%', top: '90%' }">
              <div class="start-marker">🏁</div>
              <span class="start-label">Départ</span>
            </div>
          </div>
        </div>
        
        <!-- Informations de Route -->
        <div class="route-info">
          <div class="route-stats">
            <div class="stat-item">
              <span class="stat-label">Quartiers Sélectionnés:</span>
              <span class="stat-value">{{ selectedQuartiers.length }}</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Points Potentiels:</span>
              <span class="stat-value">{{ totalPotentialPoints }}pts</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Distance Estimée:</span>
              <span class="stat-value">{{ estimatedDistance }}m</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Temps Estimé:</span>
              <span class="stat-value">{{ estimatedTime }}s</span>
            </div>
          </div>
        </div>
      </div>
    </div>
    
    <!-- Section Planification d'Itinéraire -->
    <div class="card route-planning">
      <div class="card-header">
        <h3 class="card-title">Planification d'Itinéraire</h3>
        <div class="route-actions">
          <button class="btn btn-primary" @click="deployRoute" :disabled="selectedQuartiers.length === 0">
            🚀 Déployer Itinéraire
          </button>
          <button class="btn btn-secondary" @click="saveRoute">
            💾 Sauvegarder
          </button>
        </div>
      </div>
      
      <div class="route-sequence">
        <h4>Séquence de Visite</h4>
        <div class="sequence-list">
          <div v-for="(quartier, index) in orderedQuartiers" :key="quartier.id" 
               class="sequence-item" draggable="true" 
               @dragstart="dragStart(index)" @dragover.prevent @drop="dragDrop(index)">
            <div class="sequence-number">{{ index + 1 }}</div>
            <div class="sequence-info">
              <div class="sequence-name">
                <span class="quartier-icon">{{ getQuartierIcon(quartier.type) }}</span>
                {{ quartier.name }}
              </div>
              <div class="sequence-details">
                {{ quartier.type }} • {{ quartier.quantity }} déchets • {{ quartier.points }}pts
              </div>
            </div>
            <div class="sequence-controls">
              <button class="btn btn-outline btn-sm" @click="moveUp(index)" :disabled="index === 0">
                ⬆️
              </button>
              <button class="btn btn-outline btn-sm" @click="moveDown(index)" :disabled="index === orderedQuartiers.length - 1">
                ⬇️
              </button>
            </div>
          </div>
        </div>
      </div>
      
      <!-- Bonus Zones -->
      <div class="bonus-zones">
        <h4>Zones Bonus</h4>
        <div class="bonus-list">
          <div class="bonus-item">
            <div class="bonus-icon">🎯</div>
            <div class="bonus-info">
              <div class="bonus-name">Zone Totalement Assainie</div>
              <div class="bonus-description">+20 points si tous les déchets d'une zone sont collectés</div>
            </div>
            <div class="bonus-status">Possible</div>
          </div>
          
          <div class="bonus-item">
            <div class="bonus-icon">⏰</div>
            <div class="bonus-info">
              <div class="bonus-name">Bonus Temps</div>
              <div class="bonus-description">Points supplémentaires selon le temps restant</div>
            </div>
            <div class="bonus-status">Variable</div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import { ref, computed, onMounted } from 'vue'
import ROSService from '../services/ROSService.js'

export default {
  name: 'MissionPlanning',
  setup() {
    const scanning = ref(false)
    const draggedIndex = ref(null)
    
    const scannedQuartiers = ref([
      {
        id: 1,
        name: 'Q-Commercial-01',
        type: 'Commercial',
        wasteTypes: ['recyclable', 'menager'],
        quantity: 8,
        points: 24,
        x: 25,
        y: 30,
        completed: false,
        selected: true,
        routeOrder: 1
      },
      {
        id: 2,
        name: 'Q-Residentiel-02',
        type: 'Résidentiel',
        wasteTypes: ['menager', 'compostable'],
        quantity: 12,
        points: 36,
        x: 60,
        y: 25,
        completed: false,
        selected: true,
        routeOrder: 2
      },
      {
        id: 3,
        name: 'Q-Industriel-03',
        type: 'Industriel',
        wasteTypes: ['recyclable'],
        quantity: 15,
        points: 45,
        x: 75,
        y: 70,
        completed: false,
        selected: false,
        routeOrder: null
      }
    ])
    
    const selectedQuartiers = computed(() => {
      return scannedQuartiers.value.filter(q => q.selected)
    })
    
    const orderedQuartiers = computed(() => {
      return selectedQuartiers.value
        .filter(q => q.routeOrder)
        .sort((a, b) => a.routeOrder - b.routeOrder)
    })
    
    const totalPotentialPoints = computed(() => {
      return selectedQuartiers.value.reduce((sum, q) => sum + q.points, 0)
    })
    
    const estimatedDistance = computed(() => {
      if (orderedQuartiers.value.length < 2) return 0
      // Calcul simplifié de distance
      let distance = 0
      for (let i = 1; i < orderedQuartiers.value.length; i++) {
        const prev = orderedQuartiers.value[i - 1]
        const curr = orderedQuartiers.value[i]
        distance += Math.sqrt(Math.pow(curr.x - prev.x, 2) + Math.pow(curr.y - prev.y, 2))
      }
      return Math.round(distance * 10) // Conversion approximative en mètres
    })
    
    const estimatedTime = computed(() => {
      return Math.round(estimatedDistance.value * 0.5 + orderedQuartiers.value.length * 30)
    })
    
    const routePath = computed(() => {
      if (orderedQuartiers.value.length < 2) return ''
      
      let path = `M 10 90` // Point de départ
      orderedQuartiers.value.forEach(quartier => {
        path += ` L ${quartier.x} ${quartier.y}`
      })
      return path
    })
    
    // Méthodes
    const getQuartierIcon = (type) => {
      const icons = {
        'Commercial': '🏪',
        'Résidentiel': '🏠',
        'Industriel': '🏭'
      }
      return icons[type] || '📍'
    }
    
    const getWasteIcon = (type) => {
      const icons = {
        'menager': '🗑️',
        'recyclable': '♻️',
        'compostable': '🌱'
      }
      return icons[type] || '📦'
    }
    
    const getQuartierStatusClass = (quartier) => {
      if (quartier.completed) return 'status-connected'
      if (quartier.selected) return 'status-warning'
      return 'status-disconnected'
    }
    
    const getQuartierStatusText = (quartier) => {
      if (quartier.completed) return 'Terminé'
      if (quartier.selected) return 'Planifié'
      return 'Non planifié'
    }
    
    const startQRScan = async () => {
      scanning.value = true
      
      try {
        if (ROSService.isConnected()) {
          await ROSService.callService('startQRScan')
        }
        
        // Simuler l'ajout de nouveaux quartiers
        setTimeout(() => {
          const newQuartier = {
            id: Date.now(),
            name: `Q-Nouveau-${Math.floor(Math.random() * 100)}`,
            type: ['Commercial', 'Résidentiel', 'Industriel'][Math.floor(Math.random() * 3)],
            wasteTypes: ['menager', 'recyclable', 'compostable'].slice(0, Math.floor(Math.random() * 3) + 1),
            quantity: Math.floor(Math.random() * 20) + 5,
            points: 0,
            x: Math.random() * 80 + 10,
            y: Math.random() * 60 + 20,
            completed: false,
            selected: false,
            routeOrder: null
          }
          newQuartier.points = newQuartier.quantity * 3
          scannedQuartiers.value.push(newQuartier)
          scanning.value = false
        }, 3000)
        
      } catch (error) {
        console.error('Erreur scan QR:', error)
        scanning.value = false
      }
    }
    
    const clearScannedQuartiers = () => {
      if (confirm('Effacer tous les quartiers scannés ?')) {
        scannedQuartiers.value = []
      }
    }
    
    const toggleQuartierSelection = (quarterId) => {
      const quartier = scannedQuartiers.value.find(q => q.id === quarterId)
      if (quartier && !quartier.completed) {
        quartier.selected = !quartier.selected
        if (quartier.selected) {
          const maxOrder = Math.max(0, ...selectedQuartiers.value.map(q => q.routeOrder || 0))
          quartier.routeOrder = maxOrder + 1
        } else {
          quartier.routeOrder = null
        }
        updateRouteNumbers()
      }
    }
    
    const updateRouteNumbers = () => {
      const ordered = selectedQuartiers.value.filter(q => q.routeOrder).sort((a, b) => a.routeOrder - b.routeOrder)
      ordered.forEach((quartier, index) => {
        quartier.routeOrder = index + 1
      })
    }
    
    const optimizeRoute = () => {
      // Algorithme simplifié d'optimisation (plus proche voisin)
      const selected = selectedQuartiers.value
      if (selected.length < 2) return
      
      let optimized = [selected[0]]
      let remaining = selected.slice(1)
      
      while (remaining.length > 0) {
        const current = optimized[optimized.length - 1]
        let closest = remaining[0]
        let minDistance = getDistance(current, closest)
        
        remaining.forEach(quartier => {
          const distance = getDistance(current, quartier)
          if (distance < minDistance) {
            closest = quartier
            minDistance = distance
          }
        })
        
        optimized.push(closest)
        remaining = remaining.filter(q => q.id !== closest.id)
      }
      
      optimized.forEach((quartier, index) => {
        quartier.routeOrder = index + 1
      })
    }
    
    const getDistance = (q1, q2) => {
      return Math.sqrt(Math.pow(q2.x - q1.x, 2) + Math.pow(q2.y - q1.y, 2))
    }
    
    const resetRoute = () => {
      selectedQuartiers.value.forEach(quartier => {
        quartier.selected = false
        quartier.routeOrder = null
      })
    }
    
    const deployRoute = async () => {
      if (selectedQuartiers.value.length === 0) return
      
      if (confirm(`Déployer l'itinéraire avec ${selectedQuartiers.value.length} quartiers ?`)) {
        try {
          for (const quartier of orderedQuartiers.value) {
            if (ROSService.isConnected()) {
              await ROSService.callService('goToTarget', {
                x: quartier.x / 100, // Conversion en coordonnées réelles
                y: quartier.y / 100,
                name: quartier.name
              })
            }
          }
          console.log('Itinéraire déployé avec succès')
        } catch (error) {
          console.error('Erreur déploiement:', error)
        }
      }
    }
    
    const saveRoute = () => {
      const routeName = prompt('Nom de l\'itinéraire:')
      if (routeName) {
        const routeData = {
          name: routeName,
          quartiers: orderedQuartiers.value,
          totalPoints: totalPotentialPoints.value,
          estimatedTime: estimatedTime.value
        }
        localStorage.setItem(`route_${Date.now()}`, JSON.stringify(routeData))
        console.log('Itinéraire sauvegardé:', routeName)
      }
    }
    
    // Drag & Drop
    const dragStart = (index) => {
      draggedIndex.value = index
    }
    
    const dragDrop = (dropIndex) => {
      if (draggedIndex.value !== null && draggedIndex.value !== dropIndex) {
        const draggedItem = orderedQuartiers.value[draggedIndex.value]
        const newOrder = [...orderedQuartiers.value]
        newOrder.splice(draggedIndex.value, 1)
        newOrder.splice(dropIndex, 0, draggedItem)
        
        newOrder.forEach((quartier, index) => {
          quartier.routeOrder = index + 1
        })
      }
      draggedIndex.value = null
    }
    
    const moveUp = (index) => {
      if (index > 0) {
        const newOrder = [...orderedQuartiers.value]
        ;[newOrder[index], newOrder[index - 1]] = [newOrder[index - 1], newOrder[index]]
        newOrder.forEach((quartier, i) => {
          quartier.routeOrder = i + 1
        })
      }
    }
    
    const moveDown = (index) => {
      if (index < orderedQuartiers.value.length - 1) {
        const newOrder = [...orderedQuartiers.value]
        ;[newOrder[index], newOrder[index + 1]] = [newOrder[index + 1], newOrder[index]]
        newOrder.forEach((quartier, i) => {
          quartier.routeOrder = i + 1
        })
      }
    }
    
    onMounted(() => {
      // S'abonner aux résultats de scan QR
      if (ROSService.isConnected()) {
        ROSService.subscribe('qrScanResult', (message) => {
          const newQuartier = {
            id: Date.now(),
            name: message.name,
            type: message.type,
            wasteTypes: message.waste_types,
            quantity: message.quantity,
            points: message.points,
            x: message.x || Math.random() * 80 + 10,
            y: message.y || Math.random() * 60 + 20,
            completed: false,
            selected: false,
            routeOrder: null
          }
          scannedQuartiers.value.push(newQuartier)
        })
      }
    })
    
    return {
      scanning,
      scannedQuartiers,
      selectedQuartiers,
      orderedQuartiers,
      totalPotentialPoints,
      estimatedDistance,
      estimatedTime,
      routePath,
      getQuartierIcon,
      getWasteIcon,
      getQuartierStatusClass,
      getQuartierStatusText,
      startQRScan,
      clearScannedQuartiers,
      toggleQuartierSelection,
      optimizeRoute,
      resetRoute,
      deployRoute,
      saveRoute,
      dragStart,
      dragDrop,
      moveUp,
      moveDown
    }
  }
}
</script>

<style scoped>
.mission-planning-container {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
  height: 100%;
}

.planning-header {
  text-align: center;
  margin-bottom: var(--spacing-lg);
}

.planning-header h2 {
  color: var(--text-primary);
  margin-bottom: var(--spacing-sm);
}

.planning-header p {
  color: var(--text-secondary);
}

.planning-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: var(--spacing-lg);
  flex: 1;
  min-height: 400px;
}

/* Section Scan QR */
.qr-scan-section {
  display: flex;
  flex-direction: column;
}

.scan-controls {
  display: flex;
  gap: var(--spacing-sm);
}

.scanned-quartiers-table {
  flex: 1;
  overflow: auto;
}

.quartiers-table {
  width: 100%;
  border-collapse: collapse;
  font-size: 0.875rem;
}

.quartiers-table th {
  background: var(--bg-surface);
  padding: var(--spacing-sm);
  text-align: left;
  font-weight: 600;
  color: var(--text-secondary);
  border-bottom: 1px solid var(--border-color);
}

.quartiers-table td {
  padding: var(--spacing-sm);
  border-bottom: 1px solid var(--border-color);
  vertical-align: middle;
}

.quartiers-table tr {
  cursor: pointer;
  transition: background-color 0.2s;
}

.quartiers-table tr:hover {
  background: var(--bg-surface);
}

.quartiers-table tr.selected {
  background: rgba(37, 99, 235, 0.1);
  border-left: 3px solid var(--primary-color);
}

.quartiers-table tr.completed {
  opacity: 0.6;
  background: rgba(16, 185, 129, 0.1);
}

.quartier-name {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-weight: 500;
}

.quartier-icon {
  font-size: 1rem;
}

.waste-types {
  display: flex;
  gap: var(--spacing-xs);
}

.waste-tag {
  display: inline-block;
  padding: 2px 4px;
  border-radius: var(--border-radius-sm);
  font-size: 0.75rem;
}

.waste-tag.menager { background: rgba(239, 68, 68, 0.2); }
.waste-tag.recyclable { background: rgba(16, 185, 129, 0.2); }
.waste-tag.compostable { background: rgba(245, 158, 11, 0.2); }

.quantity {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  text-align: center;
}

.points {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--secondary-color);
  text-align: center;
}

.empty-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: var(--spacing-2xl);
  color: var(--text-muted);
}

.empty-icon {
  font-size: 3rem;
  margin-bottom: var(--spacing-md);
}

/* Section Carte Stratégique */
.strategy-map-section {
  display: flex;
  flex-direction: column;
}

.map-tools {
  display: flex;
  gap: var(--spacing-xs);
}

.strategy-map-container {
  flex: 1;
  position: relative;
  min-height: 300px;
  margin-bottom: var(--spacing-lg);
}

.arena-map-planning {
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

.map-quartier {
  position: absolute;
  transform: translate(-50%, -50%);
  cursor: pointer;
  z-index: 5;
}

.quartier-marker {
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2px;
}

.marker-icon {
  font-size: 1.5rem;
  filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.3));
}

.marker-label {
  font-size: 0.7rem;
  background: var(--bg-card);
  padding: 2px 6px;
  border-radius: 4px;
  border: 1px solid var(--border-color);
  white-space: nowrap;
}

.map-quartier.selected .marker-label {
  background: var(--primary-color);
  color: white;
  border-color: var(--primary-color);
}

.route-number {
  position: absolute;
  top: -5px;
  right: -5px;
  width: 20px;
  height: 20px;
  background: var(--primary-color);
  color: white;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.75rem;
  font-weight: 600;
}

.route-lines {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
  z-index: 1;
}

.start-position {
  position: absolute;
  transform: translate(-50%, -50%);
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
  z-index: 10;
}

.start-marker {
  font-size: 1.5rem;
  filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.3));
}

.start-label {
  font-size: 0.7rem;
  background: var(--secondary-color);
  color: white;
  padding: 2px 6px;
  border-radius: 4px;
  font-weight: 600;
}

.route-info {
  background: var(--bg-surface);
  padding: var(--spacing-md);
  border-radius: var(--border-radius-md);
}

.route-stats {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: var(--spacing-md);
}

.stat-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.stat-label {
  font-size: 0.875rem;
  color: var(--text-secondary);
}

.stat-value {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--text-primary);
}

/* Section Planification */
.route-planning {
  grid-column: 1 / -1;
}

.route-actions {
  display: flex;
  gap: var(--spacing-md);
}

.route-sequence {
  margin-bottom: var(--spacing-xl);
}

.route-sequence h4 {
  color: var(--text-secondary);
  margin-bottom: var(--spacing-md);
}

.sequence-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.sequence-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-md);
  background: var(--bg-surface);
  border-radius: var(--border-radius-md);
  cursor: move;
}

.sequence-item:hover {
  background: var(--bg-tertiary);
}

.sequence-number {
  width: 30px;
  height: 30px;
  background: var(--primary-color);
  color: white;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 600;
  font-size: 0.875rem;
}

.sequence-info {
  flex: 1;
}

.sequence-name {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-weight: 500;
  margin-bottom: 2px;
}

.sequence-details {
  font-size: 0.75rem;
  color: var(--text-muted);
}

.sequence-controls {
  display: flex;
  gap: var(--spacing-xs);
}

.bonus-zones {
  background: var(--bg-surface);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-md);
}

.bonus-zones h4 {
  color: var(--text-secondary);
  margin-bottom: var(--spacing-md);
}

.bonus-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
}

.bonus-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
}

.bonus-icon {
  font-size: 1.5rem;
}

.bonus-info {
  flex: 1;
}

.bonus-name {
  font-weight: 500;
  margin-bottom: 2px;
}

.bonus-description {
  font-size: 0.75rem;
  color: var(--text-muted);
}

.bonus-status {
  padding: var(--spacing-xs) var(--spacing-sm);
  background: rgba(16, 185, 129, 0.2);
  color: var(--secondary-color);
  border-radius: var(--border-radius-sm);
  font-size: 0.75rem;
  font-weight: 600;
}
</style>