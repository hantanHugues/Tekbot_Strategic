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
          <h3 class="card-title">Quartiers QR Scannés</h3>
          <div class="scan-controls">
            <button class="btn btn-primary" @click="startQRScan" :disabled="scanning">
              {{ scanning ? '🔄 Scan en cours...' : '📱 Démarrer Scan' }}
            </button>
            <button class="btn btn-secondary" @click="clearScannedQuartiers">
              🗑️ Effacer
            </button>
            <button class="btn btn-outline" @click="importQuartiers">
              📥 Importer
            </button>
            <button class="btn btn-outline" @click="exportQuartiers">
              📤 Exporter
            </button>
          </div>
        </div>
        
        <!-- Filtres et recherche -->
        <div class="table-filters">
          <div class="filter-group">
            <input 
              v-model="searchFilter" 
              class="search-input" 
              placeholder="🔍 Rechercher un quartier..."
            />
            <select v-model="typeFilter" class="type-filter">
              <option value="all">Tous les types</option>
              <option value="Commercial">Commercial</option>
              <option value="Résidentiel">Résidentiel</option>
              <option value="Industriel">Industriel</option>
            </select>
            <select v-model="statusFilter" class="status-filter">
              <option value="all">Tous statuts</option>
              <option value="planned">Planifiés</option>
              <option value="completed">Terminés</option>
              <option value="available">Disponibles</option>
            </select>
          </div>
        </div>
        
        <div class="scanned-quartiers-table">
          <div class="table-stats">
            <div class="stat-item">
              <span class="stat-label">Total Quartiers:</span>
              <span class="stat-value">{{ filteredQuartiers.length }}</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Sélectionnés:</span>
              <span class="stat-value">{{ selectedQuartiers.length }}</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Points Max:</span>
              <span class="stat-value">{{ maxPossiblePoints }}pts</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Efficacité:</span>
              <span class="stat-value">{{ routeEfficiency }}%</span>
            </div>
          </div>
          
          <table class="quartiers-table">
            <thead>
              <tr>
                <th @click="sortBy('name')" class="sortable">
                  Nom {{ getSortIcon('name') }}
                </th>
                <th @click="sortBy('type')" class="sortable">
                  Type {{ getSortIcon('type') }}
                </th>
                <th>Déchets</th>
                <th @click="sortBy('quantity')" class="sortable">
                  Quantité {{ getSortIcon('quantity') }}
                </th>
                <th @click="sortBy('points')" class="sortable">
                  Points {{ getSortIcon('points') }}
                </th>
                <th @click="sortBy('priority')" class="sortable">
                  Priorité {{ getSortIcon('priority') }}
                </th>
                <th>Temps</th>
                <th>Statut</th>
                <th>Actions</th>
              </tr>
            </thead>
            <tbody>
              <tr v-for="quartier in paginatedQuartiers" :key="quartier.id" 
                  :class="{ completed: quartier.completed, selected: quartier.selected, urgent: quartier.priority === 'urgent' }"
                  @click="toggleQuartierSelection(quartier.id)">
                <td class="quartier-name">
                  <span class="quartier-icon">{{ getQuartierIcon(quartier.type) }}</span>
                  <div class="name-details">
                    <div class="name-primary">{{ quartier.name }}</div>
                    <div class="name-secondary">{{ quartier.qrCode }}</div>
                  </div>
                </td>
                <td>
                  <div class="type-badge" :class="quartier.type.toLowerCase()">
                    {{ quartier.type }}
                  </div>
                </td>
                <td>
                  <div class="waste-types">
                    <span v-for="waste in quartier.wasteTypes" :key="waste" 
                          class="waste-tag" :class="waste">
                      {{ getWasteIcon(waste) }}
                      <span class="waste-count">{{ quartier.wasteDetails[waste] || 0 }}</span>
                    </span>
                  </div>
                </td>
                <td class="quantity">
                  <div class="quantity-display">
                    <span class="quantity-value">{{ quartier.quantity }}</span>
                    <span class="quantity-weight">{{ quartier.estimatedWeight }}kg</span>
                  </div>
                </td>
                <td class="points">
                  <div class="points-display">
                    <span class="base-points">{{ quartier.basePoints }}</span>
                    <span class="bonus-points" v-if="quartier.bonusPoints">+{{ quartier.bonusPoints }}</span>
                    <span class="total-points">= {{ quartier.points }}pts</span>
                  </div>
                </td>
                <td class="priority">
                  <div class="priority-indicator" :class="quartier.priority">
                    {{ getPriorityIcon(quartier.priority) }}
                    {{ quartier.priority }}
                  </div>
                </td>
                <td class="time-estimate">
                  <div class="time-info">
                    <span class="travel-time">{{ quartier.travelTime }}s</span>
                    <span class="work-time">+{{ quartier.workTime }}s</span>
                  </div>
                </td>
                <td>
                  <div class="status-indicator" :class="getQuartierStatusClass(quartier)">
                    <div class="status-dot"></div>
                    {{ getQuartierStatusText(quartier) }}
                  </div>
                </td>
                <td class="actions">
                  <button class="btn-mini" @click.stop="viewQuartierDetails(quartier)" title="Détails">
                    👁️
                  </button>
                  <button class="btn-mini" @click.stop="editQuartier(quartier)" title="Modifier">
                    ✏️
                  </button>
                  <button class="btn-mini danger" @click.stop="deleteQuartier(quartier.id)" title="Supprimer">
                    🗑️
                  </button>
                </td>
              </tr>
            </tbody>
          </table>
          
          <!-- Pagination -->
          <div class="table-pagination" v-if="totalPages > 1">
            <button class="btn btn-outline btn-sm" @click="currentPage--" :disabled="currentPage === 1">
              ⬅️ Précédent
            </button>
            <span class="page-info">
              Page {{ currentPage }} sur {{ totalPages }}
            </span>
            <button class="btn btn-outline btn-sm" @click="currentPage++" :disabled="currentPage === totalPages">
              Suivant ➡️
            </button>
          </div>
          
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
          <h3 class="card-title">Carte Stratégique Interactive</h3>
          <div class="map-tools">
            <select v-model="selectedStrategy" class="strategy-selector">
              <option value="efficiency">Efficacité Max</option>
              <option value="points">Points Max</option>
              <option value="time">Temps Min</option>
              <option value="safety">Sécurité Max</option>
            </select>
            <button class="btn btn-outline btn-sm" @click="optimizeRoute">
              ⚡ Optimiser Route
            </button>
            <button class="btn btn-outline btn-sm" @click="analyzeRoute">
              📈 Analyser
            </button>
            <button class="btn btn-outline btn-sm" @click="resetRoute">
              🔄 Reset
            </button>
            <button class="btn btn-outline btn-sm" @click="toggleMapMode">
              {{ mapMode === '2D' ? '🌍 3D' : '🗺️ 2D' }}
            </button>
          </div>
        </div>
        
        <!-- Légende de la carte -->
        <div class="map-legend">
          <div class="legend-item">
            <div class="legend-color" style="background: #10b981;"></div>
            <span>Zone sûre</span>
          </div>
          <div class="legend-item">
            <div class="legend-color" style="background: #f59e0b;"></div>
            <span>Zone attention</span>
          </div>
          <div class="legend-item">
            <div class="legend-color" style="background: #ef4444;"></div>
            <span>Zone dangereuse</span>
          </div>
          <div class="legend-item">
            <div class="legend-color" style="background: #8b5cf6;"></div>
            <span>Zone bonus</span>
          </div>
        </div>
        
        <div class="strategy-map-container">
          <div class="arena-map-planning" :class="mapMode.toLowerCase()">
            <!-- Zones de sécurité -->
            <div class="safety-zones">
              <div class="safety-zone safe" :style="{ left: '5%', top: '5%', width: '30%', height: '25%' }"></div>
              <div class="safety-zone warning" :style="{ left: '40%', top: '15%', width: '25%', height: '20%' }"></div>
              <div class="safety-zone danger" :style="{ left: '70%', top: '30%', width: '25%', height: '30%' }"></div>
              <div class="safety-zone bonus" :style="{ left: '20%', top: '70%', width: '35%', height: '25%' }"></div>
            </div>
            
            <!-- Grille de navigation -->
            <div class="navigation-grid">
              <div v-for="(row, i) in navigationGrid" :key="i" class="grid-row">
                <div v-for="(cell, j) in row" :key="j" 
                     class="grid-cell" 
                     :class="cell.type"
                     @click="toggleGridCell(i, j)"
                     :style="{ left: (j * 10) + '%', top: (i * 10) + '%' }">
                </div>
              </div>
            </div>
            
            <!-- Quartiers sur la carte -->
            <div v-for="(quartier, index) in scannedQuartiers" :key="quartier.id"
                 class="map-quartier" 
                 :class="{ 
                   selected: quartier.selected, 
                   completed: quartier.completed,
                   urgent: quartier.priority === 'urgent',
                   optimal: quartier.isOptimal
                 }"
                 :style="{ left: quartier.x + '%', top: quartier.y + '%' }"
                 @click="toggleQuartierSelection(quartier.id)"
                 @mouseenter="showQuartierTooltip(quartier, $event)"
                 @mouseleave="hideQuartierTooltip">
              <div class="quartier-marker">
                <div class="marker-ring" v-if="quartier.selected"></div>
                <span class="marker-icon">{{ getQuartierIcon(quartier.type) }}</span>
                <span class="marker-label">{{ quartier.name }}</span>
                <div v-if="quartier.routeOrder" class="route-number">{{ quartier.routeOrder }}</div>
                <div class="quartier-stats">
                  <span class="stat-points">{{ quartier.points }}pts</span>
                  <span class="stat-time">{{ quartier.travelTime }}s</span>
                </div>
              </div>
            </div>
            
            <!-- Lignes de route optimisée -->
            <svg class="route-lines" viewBox="0 0 100 100">
              <!-- Route principale -->
              <path v-if="routePath" :d="routePath" stroke="#2563eb" stroke-width="0.8" 
                    fill="none" stroke-dasharray="3,2" opacity="0.9"/>
              <!-- Route alternative -->
              <path v-if="alternativeRoutePath" :d="alternativeRoutePath" stroke="#f59e0b" stroke-width="0.6" 
                    fill="none" stroke-dasharray="2,3" opacity="0.6"/>
              <!-- Zones d'évitement -->
              <circle v-for="hazard in hazardZones" :key="hazard.id"
                     :cx="hazard.x" :cy="hazard.y" :r="hazard.radius"
                     fill="rgba(239, 68, 68, 0.3)" stroke="#ef4444" stroke-width="0.2"/>
            </svg>
            
            <!-- Position de départ -->
            <div class="start-position" :style="{ left: '10%', top: '90%' }">
              <div class="start-marker" :class="{ active: routeActive }">🏁</div>
              <span class="start-label">Base</span>
            </div>
            
            <!-- Position finale -->
            <div class="end-position" :style="{ left: '85%', top: '10%' }">
              <div class="end-marker">🏆</div>
              <span class="end-label">Arrivée</span>
            </div>
            
            <!-- Robot en temps réel -->
            <div v-if="showRealTimeRobot" 
                 class="robot-position" 
                 :style="{ left: robotPosition.x + '%', top: robotPosition.y + '%' }">
              <div class="robot-marker" :class="{ moving: robotPosition.moving }">
                🤖
              </div>
              <div class="robot-trail"></div>
            </div>
          </div>
        </div>
        
        <!-- Tooltip des quartiers -->
        <div v-if="tooltipData" class="quartier-tooltip" :style="tooltipStyle">
          <h4>{{ tooltipData.name }}</h4>
          <p><strong>Type:</strong> {{ tooltipData.type }}</p>
          <p><strong>Déchets:</strong> {{ tooltipData.quantity }} éléments</p>
          <p><strong>Points:</strong> {{ tooltipData.points }}pts</p>
          <p><strong>Temps d'accès:</strong> {{ tooltipData.travelTime }}s</p>
          <p><strong>Priorité:</strong> {{ tooltipData.priority }}</p>
        </div>
        
        <!-- Informations de Route Améliorées -->
        <div class="route-info">
          <div class="route-stats-grid">
            <div class="stat-card primary">
              <div class="stat-icon">🎯</div>
              <div class="stat-content">
                <span class="stat-value">{{ selectedQuartiers.length }}</span>
                <span class="stat-label">Quartiers Sélectionnés</span>
                <span class="stat-sublabel">/ {{ scannedQuartiers.length }} total</span>
              </div>
            </div>
            
            <div class="stat-card success">
              <div class="stat-icon">⭐</div>
              <div class="stat-content">
                <span class="stat-value">{{ totalPotentialPoints }}</span>
                <span class="stat-label">Points Potentiels</span>
                <span class="stat-sublabel">{{ routeEfficiencyText }}</span>
              </div>
            </div>
            
            <div class="stat-card warning">
              <div class="stat-icon">📏</div>
              <div class="stat-content">
                <span class="stat-value">{{ estimatedDistance }}m</span>
                <span class="stat-label">Distance</span>
                <span class="stat-sublabel">{{ estimatedDistance > 100 ? 'Long parcours' : 'Parcours court' }}</span>
              </div>
            </div>
            
            <div class="stat-card info">
              <div class="stat-icon">⏱️</div>
              <div class="stat-content">
                <span class="stat-value">{{ formatTime(estimatedTime) }}</span>
                <span class="stat-label">Temps Total</span>
                <span class="stat-sublabel">{{ timeStatusText }}</span>
              </div>
            </div>
            
            <div class="stat-card danger" v-if="riskLevel > 0">
              <div class="stat-icon">⚠️</div>
              <div class="stat-content">
                <span class="stat-value">{{ riskLevel }}%</span>
                <span class="stat-label">Niveau de Risque</span>
                <span class="stat-sublabel">{{ riskStatusText }}</span>
              </div>
            </div>
            
            <div class="stat-card special">
              <div class="stat-icon">🏆</div>
              <div class="stat-content">
                <span class="stat-value">{{ bonusOpportunities }}</span>
                <span class="stat-label">Bonus Possibles</span>
                <span class="stat-sublabel">+{{ potentialBonusPoints }}pts max</span>
              </div>
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
    
    // Filtres et tri
    const searchFilter = ref('')
    const typeFilter = ref('all')
    const statusFilter = ref('all')
    const sortColumn = ref('name')
    const sortDirection = ref('asc')
    const currentPage = ref(1)
    const itemsPerPage = ref(10)
    
    // Carte stratégique
    const selectedStrategy = ref('efficiency')
    const mapMode = ref('2D')
    const showRealTimeRobot = ref(false)
    const tooltipData = ref(null)
    const tooltipStyle = ref({})
    const routeActive = ref(false)
    
    // Grille de navigation
    const navigationGrid = ref(Array(10).fill().map(() => Array(10).fill({ type: 'free', cost: 1 })))
    
    // Zones de danger
    const hazardZones = ref([
      { id: 1, x: 75, y: 45, radius: 8, type: 'obstacle' },
      { id: 2, x: 45, y: 25, radius: 5, type: 'slow' }
    ])
    
    // Position robot temps réel
    const robotPosition = ref({ x: 10, y: 90, moving: false })
    
    const scannedQuartiers = ref([
      {
        id: 1,
        name: 'Q-Commercial-01',
        qrCode: 'QR-COM-001',
        type: 'Commercial',
        wasteTypes: ['recyclable', 'menager'],
        wasteDetails: { recyclable: 5, menager: 3 },
        quantity: 8,
        estimatedWeight: 12.5,
        basePoints: 20,
        bonusPoints: 4,
        points: 24,
        priority: 'normal',
        x: 25,
        y: 30,
        travelTime: 35,
        workTime: 45,
        completed: false,
        selected: true,
        routeOrder: 1,
        isOptimal: true
      },
      {
        id: 2,
        name: 'Q-Residentiel-02',
        qrCode: 'QR-RES-002',
        type: 'Résidentiel',
        wasteTypes: ['menager', 'compostable'],
        wasteDetails: { menager: 7, compostable: 5 },
        quantity: 12,
        estimatedWeight: 18.2,
        basePoints: 30,
        bonusPoints: 6,
        points: 36,
        priority: 'high',
        x: 60,
        y: 25,
        travelTime: 25,
        workTime: 60,
        completed: false,
        selected: true,
        routeOrder: 2,
        isOptimal: true
      },
      {
        id: 3,
        name: 'Q-Industriel-03',
        qrCode: 'QR-IND-003',
        type: 'Industriel',
        wasteTypes: ['recyclable'],
        wasteDetails: { recyclable: 15 },
        quantity: 15,
        estimatedWeight: 22.8,
        basePoints: 40,
        bonusPoints: 5,
        points: 45,
        priority: 'urgent',
        x: 75,
        y: 70,
        travelTime: 45,
        workTime: 90,
        completed: false,
        selected: false,
        routeOrder: null,
        isOptimal: false
      },
      {
        id: 4,
        name: 'Q-Mixed-04',
        qrCode: 'QR-MIX-004',
        type: 'Commercial',
        wasteTypes: ['recyclable', 'menager', 'compostable'],
        wasteDetails: { recyclable: 4, menager: 3, compostable: 2 },
        quantity: 9,
        estimatedWeight: 14.1,
        basePoints: 25,
        bonusPoints: 8,
        points: 33,
        priority: 'normal',
        x: 40,
        y: 55,
        travelTime: 30,
        workTime: 55,
        completed: false,
        selected: false,
        routeOrder: null,
        isOptimal: true
      },
      {
        id: 5,
        name: 'Q-Bonus-05',
        qrCode: 'QR-BON-005',
        type: 'Résidentiel',
        wasteTypes: ['compostable'],
        wasteDetails: { compostable: 10 },
        quantity: 10,
        estimatedWeight: 8.5,
        basePoints: 15,
        bonusPoints: 15,
        points: 30,
        priority: 'low',
        x: 20,
        y: 75,
        travelTime: 20,
        workTime: 40,
        completed: false,
        selected: false,
        routeOrder: null,
        isOptimal: false
      }
    ])
    
    // Computed properties avancées
    const filteredQuartiers = computed(() => {
      let filtered = scannedQuartiers.value
      
      // Filtre par recherche
      if (searchFilter.value) {
        const search = searchFilter.value.toLowerCase()
        filtered = filtered.filter(q => 
          q.name.toLowerCase().includes(search) ||
          q.qrCode.toLowerCase().includes(search) ||
          q.type.toLowerCase().includes(search)
        )
      }
      
      // Filtre par type
      if (typeFilter.value !== 'all') {
        filtered = filtered.filter(q => q.type === typeFilter.value)
      }
      
      // Filtre par statut
      if (statusFilter.value !== 'all') {
        switch(statusFilter.value) {
          case 'planned':
            filtered = filtered.filter(q => q.selected && !q.completed)
            break
          case 'completed':
            filtered = filtered.filter(q => q.completed)
            break
          case 'available':
            filtered = filtered.filter(q => !q.selected && !q.completed)
            break
        }
      }
      
      // Tri
      filtered.sort((a, b) => {
        let aVal = a[sortColumn.value]
        let bVal = b[sortColumn.value]
        
        if (typeof aVal === 'string') {
          aVal = aVal.toLowerCase()
          bVal = bVal.toLowerCase()
        }
        
        if (sortDirection.value === 'asc') {
          return aVal < bVal ? -1 : aVal > bVal ? 1 : 0
        } else {
          return aVal > bVal ? -1 : aVal < bVal ? 1 : 0
        }
      })
      
      return filtered
    })
    
    const paginatedQuartiers = computed(() => {
      const start = (currentPage.value - 1) * itemsPerPage.value
      const end = start + itemsPerPage.value
      return filteredQuartiers.value.slice(start, end)
    })
    
    const totalPages = computed(() => {
      return Math.ceil(filteredQuartiers.value.length / itemsPerPage.value)
    })
    
    const selectedQuartiers = computed(() => {
      return scannedQuartiers.value.filter(q => q.selected)
    })
    
    const maxPossiblePoints = computed(() => {
      return scannedQuartiers.value.reduce((sum, q) => sum + q.points, 0)
    })
    
    const orderedQuartiers = computed(() => {
      return selectedQuartiers.value
        .filter(q => q.routeOrder)
        .sort((a, b) => a.routeOrder - b.routeOrder)
    })
    
    const totalPotentialPoints = computed(() => {
      return selectedQuartiers.value.reduce((sum, q) => sum + q.points, 0)
    })
    
    const routeEfficiency = computed(() => {
      if (maxPossiblePoints.value === 0) return 0
      return Math.round((totalPotentialPoints.value / maxPossiblePoints.value) * 100)
    })
    
    const routeEfficiencyText = computed(() => {
      const eff = routeEfficiency.value
      if (eff >= 80) return 'Excellent'
      if (eff >= 60) return 'Bon'
      if (eff >= 40) return 'Moyen'
      return 'À améliorer'
    })
    
    const timeStatusText = computed(() => {
      if (estimatedTime.value <= 180) return 'Rapide'
      if (estimatedTime.value <= 270) return 'Optimal'
      return 'Long'
    })
    
    const riskLevel = computed(() => {
      let risk = 0
      selectedQuartiers.value.forEach(q => {
        if (q.priority === 'urgent') risk += 15
        if (q.x > 70 && q.y < 40) risk += 10 // Zone dangereuse
        if (q.travelTime > 60) risk += 5
      })
      return Math.min(risk, 100)
    })
    
    const riskStatusText = computed(() => {
      const risk = riskLevel.value
      if (risk <= 20) return 'Faible'
      if (risk <= 50) return 'Modéré'
      if (risk <= 80) return 'Élevé'
      return 'Critique'
    })
    
    const bonusOpportunities = computed(() => {
      let count = 0
      // Zone totalement assainie
      const commercialQuartiers = selectedQuartiers.value.filter(q => q.type === 'Commercial')
      if (commercialQuartiers.length >= 2) count++
      
      // Collecte mixte
      const hasAllWasteTypes = selectedQuartiers.value.some(q => q.wasteTypes.length >= 3)
      if (hasAllWasteTypes) count++
      
      // Bonus temps
      if (estimatedTime.value < 240) count++
      
      return count
    })
    
    const potentialBonusPoints = computed(() => {
      return bonusOpportunities.value * 20
    })
    
    const alternativeRoutePath = computed(() => {
      // Calcul d'une route alternative évitant les zones à risque
      if (orderedQuartiers.value.length < 2) return ''
      // Route simplifiée pour l'exemple
      return routePath.value // À implémenter avec algorithme d'évitement
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
    
    // Méthodes utilitaires
    const formatTime = (seconds) => {
      const mins = Math.floor(seconds / 60)
      const secs = seconds % 60
      return `${mins}:${secs.toString().padStart(2, '0')}`
    }
    
    const getSortIcon = (column) => {
      if (sortColumn.value !== column) return '↕️'
      return sortDirection.value === 'asc' ? '↑' : '↓'
    }
    
    const sortBy = (column) => {
      if (sortColumn.value === column) {
        sortDirection.value = sortDirection.value === 'asc' ? 'desc' : 'asc'
      } else {
        sortColumn.value = column
        sortDirection.value = 'asc'
      }
    }
    
    const getQuartierIcon = (type) => {
      const icons = {
        'Commercial': '🏪',
        'Résidentiel': '🏠',
        'Industriel': '🏭'
      }
      return icons[type] || '📍'
    }
    
    const getPriorityIcon = (priority) => {
      const icons = {
        'low': '🟢',
        'normal': '🟡',
        'high': '🟠',
        'urgent': '🔴'
      }
      return icons[priority] || '⚫'
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
      const selected = selectedQuartiers.value
      if (selected.length < 2) return
      
      let optimized = []
      
      switch(selectedStrategy.value) {
        case 'efficiency':
          optimized = optimizeByEfficiency(selected)
          break
        case 'points':
          optimized = optimizeByPoints(selected)
          break
        case 'time':
          optimized = optimizeByTime(selected)
          break
        case 'safety':
          optimized = optimizeBySafety(selected)
          break
        default:
          optimized = optimizeByDistance(selected)
      }
      
      optimized.forEach((quartier, index) => {
        quartier.routeOrder = index + 1
        quartier.isOptimal = true
      })
    }
    
    const optimizeByEfficiency = (quartiers) => {
      // Points par minute de travail
      return [...quartiers].sort((a, b) => {
        const effA = a.points / (a.travelTime + a.workTime)
        const effB = b.points / (b.travelTime + b.workTime)
        return effB - effA
      })
    }
    
    const optimizeByPoints = (quartiers) => {
      return [...quartiers].sort((a, b) => b.points - a.points)
    }
    
    const optimizeByTime = (quartiers) => {
      return [...quartiers].sort((a, b) => (a.travelTime + a.workTime) - (b.travelTime + b.workTime))
    }
    
    const optimizeBySafety = (quartiers) => {
      const priorityOrder = { 'low': 0, 'normal': 1, 'high': 2, 'urgent': 3 }
      return [...quartiers].sort((a, b) => priorityOrder[a.priority] - priorityOrder[b.priority])
    }
    
    const optimizeByDistance = (quartiers) => {
      // Algorithme du plus proche voisin amélioré
      let optimized = [quartiers[0]]
      let remaining = quartiers.slice(1)
      
      while (remaining.length > 0) {
        const current = optimized[optimized.length - 1]
        let best = remaining[0]
        let bestScore = calculateRouteScore(current, best)
        
        remaining.forEach(quartier => {
          const score = calculateRouteScore(current, quartier)
          if (score > bestScore) {
            best = quartier
            bestScore = score
          }
        })
        
        optimized.push(best)
        remaining = remaining.filter(q => q.id !== best.id)
      }
      
      return optimized
    }
    
    const calculateRouteScore = (from, to) => {
      const distance = getDistance(from, to)
      const pointsRatio = to.points / 100
      const timeRatio = 100 / (to.travelTime + to.workTime)
      const priorityBonus = to.priority === 'urgent' ? 0.5 : to.priority === 'high' ? 0.3 : 0.1
      
      return (pointsRatio + timeRatio + priorityBonus) / (distance + 1)
    }
    
    const analyzeRoute = () => {
      // Analyse avancée de la route
      const analysis = {
        totalDistance: estimatedDistance.value,
        totalTime: estimatedTime.value,
        totalPoints: totalPotentialPoints.value,
        efficiency: routeEfficiency.value,
        riskLevel: riskLevel.value,
        bonusOpportunities: bonusOpportunities.value
      }
      
      console.log('Analyse de route:', analysis)
      alert(`Analyse de Route:

Distance: ${analysis.totalDistance}m
Temps: ${formatTime(analysis.totalTime)}
Points: ${analysis.totalPoints}
Efficacité: ${analysis.efficiency}%
Risque: ${analysis.riskLevel}%
Bonus possibles: ${analysis.bonusOpportunities}`)
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
          strategy: selectedStrategy.value,
          quartiers: orderedQuartiers.value,
          totalPoints: totalPotentialPoints.value,
          estimatedTime: estimatedTime.value,
          efficiency: routeEfficiency.value,
          riskLevel: riskLevel.value,
          createdAt: new Date().toISOString()
        }
        localStorage.setItem(`tekbot_route_${Date.now()}`, JSON.stringify(routeData))
        console.log('Itinéraire sauvegardé:', routeName)
      }
    }
    
    // Nouvelles méthodes
    const importQuartiers = () => {
      const input = document.createElement('input')
      input.type = 'file'
      input.accept = '.json'
      input.onchange = (e) => {
        const file = e.target.files[0]
        if (file) {
          const reader = new FileReader()
          reader.onload = (e) => {
            try {
              const data = JSON.parse(e.target.result)
              scannedQuartiers.value = [...scannedQuartiers.value, ...data]
              console.log('Quartiers importés avec succès')
            } catch (error) {
              alert('Erreur lors de l\'importation du fichier')
            }
          }
          reader.readAsText(file)
        }
      }
      input.click()
    }
    
    const exportQuartiers = () => {
      const data = JSON.stringify(scannedQuartiers.value, null, 2)
      const blob = new Blob([data], { type: 'application/json' })
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = `tekbot-quartiers-${new Date().toISOString().slice(0, 10)}.json`
      a.click()
      URL.revokeObjectURL(url)
    }
    
    const viewQuartierDetails = (quartier) => {
      alert(`Détails du Quartier:

Nom: ${quartier.name}
Code QR: ${quartier.qrCode}
Type: ${quartier.type}
Quantité: ${quartier.quantity} éléments
Poids estimé: ${quartier.estimatedWeight}kg
Points: ${quartier.points} (base: ${quartier.basePoints} + bonus: ${quartier.bonusPoints})
Priorité: ${quartier.priority}
Temps d'accès: ${quartier.travelTime}s
Temps de travail: ${quartier.workTime}s`)
    }
    
    const editQuartier = (quartier) => {
      const newName = prompt('Nouveau nom:', quartier.name)
      if (newName && newName !== quartier.name) {
        quartier.name = newName
        console.log('Quartier modifié:', newName)
      }
    }
    
    const deleteQuartier = (quartierId) => {
      if (confirm('Supprimer ce quartier ?')) {
        const index = scannedQuartiers.value.findIndex(q => q.id === quartierId)
        if (index !== -1) {
          scannedQuartiers.value.splice(index, 1)
          console.log('Quartier supprimé')
        }
      }
    }
    
    const toggleMapMode = () => {
      mapMode.value = mapMode.value === '2D' ? '3D' : '2D'
    }
    
    const toggleGridCell = (row, col) => {
      const cell = navigationGrid.value[row][col]
      const types = ['free', 'blocked', 'slow', 'fast']
      const currentIndex = types.indexOf(cell.type)
      const nextIndex = (currentIndex + 1) % types.length
      cell.type = types[nextIndex]
      cell.cost = { free: 1, blocked: 999, slow: 2, fast: 0.5 }[cell.type]
    }
    
    const showQuartierTooltip = (quartier, event) => {
      tooltipData.value = quartier
      tooltipStyle.value = {
        left: event.clientX + 10 + 'px',
        top: event.clientY - 10 + 'px'
      }
    }
    
    const hideQuartierTooltip = () => {
      tooltipData.value = null
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
      // État de base
      scanning,
      scannedQuartiers,
      selectedQuartiers,
      orderedQuartiers,
      
      // Filtres et pagination
      searchFilter,
      typeFilter,
      statusFilter,
      filteredQuartiers,
      paginatedQuartiers,
      currentPage,
      totalPages,
      
      // Carte stratégique
      selectedStrategy,
      mapMode,
      showRealTimeRobot,
      tooltipData,
      tooltipStyle,
      routeActive,
      navigationGrid,
      hazardZones,
      robotPosition,
      
      // Métriques calculées
      totalPotentialPoints,
      maxPossiblePoints,
      routeEfficiency,
      routeEfficiencyText,
      estimatedDistance,
      estimatedTime,
      timeStatusText,
      riskLevel,
      riskStatusText,
      bonusOpportunities,
      potentialBonusPoints,
      routePath,
      alternativeRoutePath,
      
      // Méthodes utilitaires
      formatTime,
      getSortIcon,
      sortBy,
      getQuartierIcon,
      getWasteIcon,
      getPriorityIcon,
      getQuartierStatusClass,
      getQuartierStatusText,
      
      // Actions principales
      startQRScan,
      clearScannedQuartiers,
      toggleQuartierSelection,
      optimizeRoute,
      analyzeRoute,
      resetRoute,
      deployRoute,
      saveRoute,
      
      // Nouvelles actions
      importQuartiers,
      exportQuartiers,
      viewQuartierDetails,
      editQuartier,
      deleteQuartier,
      toggleMapMode,
      toggleGridCell,
      showQuartierTooltip,
      hideQuartierTooltip,
      
      // Drag & Drop
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

/* Nouveaux styles - Filtres et recherche */
.table-filters {
  padding: var(--spacing-md);
  background: var(--bg-surface);
  border-bottom: 1px solid var(--border-color);
}

.filter-group {
  display: flex;
  gap: var(--spacing-md);
  align-items: center;
}

.search-input,
.type-filter,
.status-filter {
  padding: var(--spacing-xs) var(--spacing-sm);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-sm);
  background: var(--bg-card);
  color: var(--text-primary);
  font-size: 0.75rem;
}

.search-input {
  flex: 1;
  min-width: 200px;
}

/* Statistiques de table */
.table-stats {
  display: flex;
  justify-content: space-between;
  padding: var(--spacing-sm);
  background: var(--bg-tertiary);
  border-bottom: 1px solid var(--border-color);
  font-size: 0.75rem;
}

.table-stats .stat-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
}

.table-stats .stat-label {
  color: var(--text-muted);
}

.table-stats .stat-value {
  font-weight: 600;
  color: var(--text-primary);
}

/* Colonnes triables */
.sortable {
  cursor: pointer;
  user-select: none;
  transition: background-color 0.2s;
}

.sortable:hover {
  background: var(--bg-tertiary);
}

/* Nouvelles colonnes de tableau */
.name-details {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.name-primary {
  font-weight: 500;
  color: var(--text-primary);
}

.name-secondary {
  font-size: 0.7rem;
  color: var(--text-muted);
  font-family: 'Monaco', 'Menlo', monospace;
}

.type-badge {
  padding: var(--spacing-xs) var(--spacing-sm);
  border-radius: var(--border-radius-sm);
  font-size: 0.7rem;
  font-weight: 500;
  text-align: center;
}

.type-badge.commercial {
  background: rgba(59, 130, 246, 0.2);
  color: #3b82f6;
}

.type-badge.résidentiel {
  background: rgba(16, 185, 129, 0.2);
  color: var(--secondary-color);
}

.type-badge.industriel {
  background: rgba(245, 158, 11, 0.2);
  color: var(--warning-color);
}

.waste-count {
  margin-left: 2px;
  font-size: 0.6rem;
  font-weight: 600;
}

.quantity-display {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2px;
}

.quantity-value {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--text-primary);
}

.quantity-weight {
  font-size: 0.7rem;
  color: var(--text-muted);
}

.points-display {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2px;
}

.base-points {
  font-size: 0.7rem;
  color: var(--text-muted);
}

.bonus-points {
  font-size: 0.7rem;
  color: var(--secondary-color);
  font-weight: 600;
}

.total-points {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--primary-color);
}

.priority-indicator {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-size: 0.7rem;
  padding: var(--spacing-xs);
  border-radius: var(--border-radius-sm);
}

.priority-indicator.low {
  background: rgba(34, 197, 94, 0.2);
  color: #22c55e;
}

.priority-indicator.normal {
  background: rgba(59, 130, 246, 0.2);
  color: #3b82f6;
}

.priority-indicator.high {
  background: rgba(245, 158, 11, 0.2);
  color: var(--warning-color);
}

.priority-indicator.urgent {
  background: rgba(239, 68, 68, 0.2);
  color: var(--danger-color);
}

.time-info {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2px;
  font-size: 0.7rem;
}

.travel-time {
  color: var(--primary-color);
  font-weight: 600;
}

.work-time {
  color: var(--text-muted);
}

.actions {
  display: flex;
  gap: var(--spacing-xs);
}

.btn-mini {
  padding: 2px 4px;
  background: transparent;
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-sm);
  cursor: pointer;
  font-size: 0.7rem;
  transition: all 0.2s;
}

.btn-mini:hover {
  background: var(--bg-tertiary);
}

.btn-mini.danger:hover {
  background: rgba(239, 68, 68, 0.2);
  border-color: var(--danger-color);
}

.table-pagination {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: var(--spacing-md);
  border-top: 1px solid var(--border-color);
}

.page-info {
  font-size: 0.75rem;
  color: var(--text-muted);
}

/* Lignes urgentes */
.quartiers-table tr.urgent {
  background: rgba(239, 68, 68, 0.1);
  border-left: 3px solid var(--danger-color);
}

/* Carte stratégique avancée */
.strategy-selector {
  padding: var(--spacing-xs) var(--spacing-sm);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-sm);
  background: var(--bg-card);
  color: var(--text-primary);
  font-size: 0.75rem;
}

.map-legend {
  display: flex;
  gap: var(--spacing-md);
  padding: var(--spacing-sm);
  background: var(--bg-surface);
  border-radius: var(--border-radius-md);
  margin-bottom: var(--spacing-md);
}

.legend-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-size: 0.75rem;
}

.legend-color {
  width: 12px;
  height: 12px;
  border-radius: 2px;
}

/* Zones de sécurité */
.safety-zones {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
  z-index: 1;
}

.safety-zone {
  position: absolute;
  border-radius: var(--border-radius-md);
  opacity: 0.3;
}

.safety-zone.safe {
  background: rgba(34, 197, 94, 0.2);
  border: 1px solid #22c55e;
}

.safety-zone.warning {
  background: rgba(245, 158, 11, 0.2);
  border: 1px solid var(--warning-color);
}

.safety-zone.danger {
  background: rgba(239, 68, 68, 0.2);
  border: 1px solid var(--danger-color);
}

.safety-zone.bonus {
  background: rgba(139, 92, 246, 0.2);
  border: 1px solid #8b5cf6;
}

/* Grille de navigation */
.navigation-grid {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  z-index: 2;
}

.grid-cell {
  position: absolute;
  width: 10%;
  height: 10%;
  border: 1px solid rgba(255, 255, 255, 0.1);
  cursor: pointer;
  transition: all 0.2s;
}

.grid-cell.blocked {
  background: rgba(239, 68, 68, 0.4);
}

.grid-cell.slow {
  background: rgba(245, 158, 11, 0.3);
}

.grid-cell.fast {
  background: rgba(34, 197, 94, 0.3);
}

/* Marqueurs de quartiers améliorés */
.map-quartier.urgent .quartier-marker {
  animation: pulse-urgent 2s infinite;
}

.map-quartier.optimal .quartier-marker {
  box-shadow: 0 0 10px rgba(59, 130, 246, 0.6);
}

.marker-ring {
  position: absolute;
  top: -8px;
  left: -8px;
  right: -8px;
  bottom: -8px;
  border: 2px solid var(--primary-color);
  border-radius: 50%;
  animation: ring-pulse 1.5s infinite;
}

.quartier-stats {
  position: absolute;
  top: 100%;
  left: 50%;
  transform: translateX(-50%);
  display: flex;
  gap: var(--spacing-xs);
  margin-top: 2px;
  font-size: 0.6rem;
}

.stat-points {
  background: var(--secondary-color);
  color: white;
  padding: 1px 4px;
  border-radius: 2px;
}

.stat-time {
  background: var(--warning-color);
  color: white;
  padding: 1px 4px;
  border-radius: 2px;
}

/* Position robot temps réel */
.robot-position {
  position: absolute;
  z-index: 10;
  transform: translate(-50%, -50%);
}

.robot-marker {
  font-size: 1.5rem;
  filter: drop-shadow(0 0 8px rgba(59, 130, 246, 0.8));
  transition: all 0.3s;
}

.robot-marker.moving {
  animation: robot-move 1s ease-in-out infinite alternate;
}

.robot-trail {
  position: absolute;
  width: 3px;
  height: 20px;
  background: linear-gradient(to bottom, rgba(59, 130, 246, 0.8), transparent);
  top: 100%;
  left: 50%;
  transform: translateX(-50%);
}

/* Positions de départ/arrivée */
.start-marker.active {
  animation: pulse-start 2s infinite;
  color: var(--secondary-color);
}

.end-position {
  position: absolute;
  transform: translate(-50%, -50%);
  text-align: center;
  z-index: 5;
}

.end-marker {
  font-size: 1.5rem;
  animation: glow 3s ease-in-out infinite alternate;
}

.end-label {
  display: block;
  font-size: 0.7rem;
  color: var(--text-primary);
  margin-top: 2px;
  background: var(--bg-card);
  padding: 2px 4px;
  border-radius: 4px;
  border: 1px solid var(--border-color);
}

/* Tooltip des quartiers */
.quartier-tooltip {
  position: fixed;
  background: var(--bg-card);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-md);
  padding: var(--spacing-md);
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  z-index: 1000;
  max-width: 250px;
  font-size: 0.75rem;
  pointer-events: none;
}

.quartier-tooltip h4 {
  margin: 0 0 var(--spacing-sm) 0;
  color: var(--text-primary);
}

.quartier-tooltip p {
  margin: var(--spacing-xs) 0;
  color: var(--text-secondary);
}

/* Statistiques de route améliorées */
.route-stats-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: var(--spacing-md);
}

.stat-card {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-md);
  border-radius: var(--border-radius-md);
  border: 1px solid var(--border-color);
  transition: all 0.2s;
}

.stat-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
}

.stat-card.primary {
  background: linear-gradient(135deg, rgba(59, 130, 246, 0.1), rgba(99, 102, 241, 0.05));
  border-color: rgba(59, 130, 246, 0.3);
}

.stat-card.success {
  background: linear-gradient(135deg, rgba(34, 197, 94, 0.1), rgba(16, 185, 129, 0.05));
  border-color: rgba(34, 197, 94, 0.3);
}

.stat-card.warning {
  background: linear-gradient(135deg, rgba(245, 158, 11, 0.1), rgba(251, 191, 36, 0.05));
  border-color: rgba(245, 158, 11, 0.3);
}

.stat-card.info {
  background: linear-gradient(135deg, rgba(6, 182, 212, 0.1), rgba(14, 165, 233, 0.05));
  border-color: rgba(6, 182, 212, 0.3);
}

.stat-card.danger {
  background: linear-gradient(135deg, rgba(239, 68, 68, 0.1), rgba(248, 113, 113, 0.05));
  border-color: rgba(239, 68, 68, 0.3);
}

.stat-card.special {
  background: linear-gradient(135deg, rgba(139, 92, 246, 0.1), rgba(168, 85, 247, 0.05));
  border-color: rgba(139, 92, 246, 0.3);
}

.stat-icon {
  font-size: 1.5rem;
  opacity: 0.8;
}

.stat-content {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.stat-card .stat-value {
  font-size: 1.25rem;
  font-weight: 700;
  color: var(--text-primary);
  font-family: 'Monaco', 'Menlo', monospace;
}

.stat-card .stat-label {
  font-size: 0.75rem;
  font-weight: 500;
  color: var(--text-secondary);
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.stat-sublabel {
  font-size: 0.7rem;
  color: var(--text-muted);
  font-style: italic;
}

/* Animations */
@keyframes pulse-urgent {
  0%, 100% { transform: scale(1); }
  50% { transform: scale(1.1); }
}

@keyframes ring-pulse {
  0% { transform: scale(1); opacity: 1; }
  100% { transform: scale(1.3); opacity: 0; }
}

@keyframes robot-move {
  0% { transform: rotate(-2deg) scale(1); }
  100% { transform: rotate(2deg) scale(1.05); }
}

@keyframes pulse-start {
  0%, 100% { transform: scale(1); filter: brightness(1); }
  50% { transform: scale(1.1); filter: brightness(1.2); }
}

@keyframes glow {
  0% { filter: brightness(1) drop-shadow(0 0 5px rgba(251, 191, 36, 0.5)); }
  100% { filter: brightness(1.2) drop-shadow(0 0 15px rgba(251, 191, 36, 0.8)); }
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