<template>
  <div class="calibration-container">
    <div class="calibration-header">
      <h2>Module de Calibration</h2>
      <p>Calibration des capteurs LDR et couleur pour la station de tri</p>
    </div>
    
    <div class="calibration-grid">
      <!-- Calibration LDR -->
      <div class="card ldr-calibration">
        <div class="card-header">
          <h3 class="card-title">Calibration Capteurs LDR</h3>
          <div class="status-indicator status-connected">
            <div class="status-dot"></div>
            Connecté
          </div>
        </div>
        
        <div class="sensor-readings">
          <div class="reading-item">
            <label>Capteur Avant:</label>
            <div class="reading-value">{{ ldrFrontValue }}</div>
          </div>
          <div class="reading-item">
            <label>Capteur Arrière:</label>
            <div class="reading-value">{{ ldrBackValue }}</div>
          </div>
        </div>
        
        <div class="calibration-controls">
          <button class="btn btn-primary" @click="calibratePresence">
            🎯 Calibrer "Présence"
          </button>
          <button class="btn btn-secondary" @click="calibrateAbsence">
            ⭕ Calibrer "Absence"
          </button>
        </div>
        
        <div class="threshold-adjustment">
          <label>Ajustement Manuel:</label>
          <div class="slider-container">
            <input type="range" v-model="ldrThreshold" min="0" max="1023" class="slider">
            <span class="threshold-value">{{ ldrThreshold }}</span>
          </div>
        </div>
      </div>
      
      <!-- Calibration Couleur -->
      <div class="card color-calibration">
        <div class="card-header">
          <h3 class="card-title">Calibration Capteur Couleur</h3>
          <div class="status-indicator status-connected">
            <div class="status-dot"></div>
            Actif
          </div>
        </div>
        
        <div class="color-readings">
          <div class="rgb-display">
            <div class="color-preview" :style="{ backgroundColor: currentColorRGB }"></div>
            <div class="color-values">
              <div class="color-value">
                <span class="color-label">R:</span>
                <span class="value">{{ colorValues.r }}</span>
              </div>
              <div class="color-value">
                <span class="color-label">G:</span>
                <span class="value">{{ colorValues.g }}</span>
              </div>
              <div class="color-value">
                <span class="color-label">B:</span>
                <span class="value">{{ colorValues.b }}</span>
              </div>
            </div>
          </div>
        </div>
        
        <div class="color-learning">
          <div class="input-group">
            <input 
              type="text" 
              v-model="newColorName" 
              placeholder="Nom de la couleur"
              class="color-input"
            >
            <button class="btn btn-accent" @click="learnColor" :disabled="!newColorName">
              🧠 Apprendre Couleur
            </button>
          </div>
        </div>
        
        <div class="learned-colors">
          <h4>Couleurs Apprises:</h4>
          <div class="color-list">
            <div 
              v-for="color in learnedColors" 
              :key="color.name"
              class="color-item"
            >
              <div class="color-swatch" :style="{ backgroundColor: color.rgb }"></div>
              <span class="color-name">{{ color.name }}</span>
              <button class="btn btn-outline btn-sm" @click="removeColor(color.name)">
                ✕
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
    
    <!-- Profils de Calibration -->
    <div class="card profiles-section">
      <div class="card-header">
        <h3 class="card-title">Profils de Calibration</h3>
        <div class="profile-controls">
          <select v-model="selectedProfile" class="profile-select">
            <option v-for="profile in calibrationProfiles" :key="profile.name" :value="profile.name">
              {{ profile.name }}
            </option>
          </select>
          <button class="btn btn-primary" @click="loadProfile">
            📥 Charger
          </button>
          <button class="btn btn-secondary" @click="saveProfile">
            💾 Sauvegarder
          </button>
        </div>
      </div>
      
      <div class="profile-info">
        <p>Gérez vos profils de calibration pour différents environnements d'éclairage</p>
      </div>
    </div>
  </div>
</template>

<script>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import ROSService from '../services/ROSService.js'

export default {
  name: 'CalibrationModule',
  setup() {
    const ldrFrontValue = ref(512)
    const ldrBackValue = ref(487)
    const ldrThreshold = ref(500)
    
    const colorValues = ref({
      r: 255,
      g: 128,
      b: 64
    })
    
    const newColorName = ref('')
    const selectedProfile = ref('Éclairage Principal')
    
    const learnedColors = ref([
      { name: 'Orange', rgb: 'rgb(255, 128, 64)' },
      { name: 'Vert', rgb: 'rgb(64, 255, 128)' },
      { name: 'Bleu', rgb: 'rgb(64, 128, 255)' }
    ])
    
    const calibrationProfiles = ref([
      { name: 'Éclairage Principal' },
      { name: 'Zone d\'Entraînement' },
      { name: 'Conditions Faibles' }
    ])
    
    const currentColorRGB = computed(() => {
      return `rgb(${colorValues.value.r}, ${colorValues.value.g}, ${colorValues.value.b})`
    })
    
    const calibratePresence = async () => {
      try {
        await ROSService.callService('calibrateLDR', { mode: 'presence' })
        console.log('Calibration présence effectuée')
      } catch (error) {
        console.error('Erreur calibration présence:', error)
      }
    }
    
    const calibrateAbsence = async () => {
      try {
        await ROSService.callService('calibrateLDR', { mode: 'absence' })
        console.log('Calibration absence effectuée')
      } catch (error) {
        console.error('Erreur calibration absence:', error)
      }
    }
    
    const learnColor = async () => {
      if (!newColorName.value) return
      
      try {
        await ROSService.callService('learnColor', {
          name: newColorName.value,
          r: colorValues.value.r,
          g: colorValues.value.g,
          b: colorValues.value.b
        })
        
        learnedColors.value.push({
          name: newColorName.value,
          rgb: currentColorRGB.value
        })
        
        newColorName.value = ''
        console.log('Couleur apprise avec succès')
      } catch (error) {
        console.error('Erreur apprentissage couleur:', error)
      }
    }
    
    const removeColor = (colorName) => {
      learnedColors.value = learnedColors.value.filter(color => color.name !== colorName)
    }
    
    const loadProfile = () => {
      console.log('Chargement du profil:', selectedProfile.value)
    }
    
    const saveProfile = () => {
      const profileName = prompt('Nom du nouveau profil:')
      if (profileName) {
        calibrationProfiles.value.push({ name: profileName })
        console.log('Profil sauvegardé:', profileName)
      }
    }
    
    onMounted(() => {
      // Simuler les valeurs des capteurs
      setInterval(() => {
        ldrFrontValue.value = 500 + Math.floor(Math.random() * 100)
        ldrBackValue.value = 480 + Math.floor(Math.random() * 100)
        
        colorValues.value = {
          r: 200 + Math.floor(Math.random() * 55),
          g: 100 + Math.floor(Math.random() * 100),
          b: 50 + Math.floor(Math.random() * 100)
        }
      }, 2000)
      
      // S'abonner aux topics ROS
      if (ROSService.isConnected()) {
        ROSService.subscribe('ldrFront', (message) => {
          ldrFrontValue.value = message.data
        })
        
        ROSService.subscribe('ldrBack', (message) => {
          ldrBackValue.value = message.data
        })
        
        ROSService.subscribe('conveyorSensorColor', (message) => {
          colorValues.value = {
            r: Math.floor(message.r * 255),
            g: Math.floor(message.g * 255),
            b: Math.floor(message.b * 255)
          }
        })
      }
    })
    
    return {
      ldrFrontValue,
      ldrBackValue,
      ldrThreshold,
      colorValues,
      currentColorRGB,
      newColorName,
      selectedProfile,
      learnedColors,
      calibrationProfiles,
      calibratePresence,
      calibrateAbsence,
      learnColor,
      removeColor,
      loadProfile,
      saveProfile
    }
  }
}
</script>

<style scoped>
.calibration-container {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
  height: 100%;
}

.calibration-header {
  text-align: center;
  margin-bottom: var(--spacing-lg);
}

.calibration-header h2 {
  color: var(--text-primary);
  margin-bottom: var(--spacing-sm);
}

.calibration-header p {
  color: var(--text-secondary);
}

.calibration-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: var(--spacing-lg);
  flex: 1;
}

.ldr-calibration,
.color-calibration {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
}

.sensor-readings,
.color-readings {
  background: var(--bg-surface);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-md);
}

.reading-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: var(--spacing-sm);
}

.reading-item:last-child {
  margin-bottom: 0;
}

.reading-item label {
  color: var(--text-secondary);
  font-weight: 500;
}

.reading-value {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--primary-color);
  background: var(--bg-tertiary);
  padding: var(--spacing-xs) var(--spacing-sm);
  border-radius: var(--border-radius-sm);
}

.calibration-controls {
  display: flex;
  gap: var(--spacing-md);
}

.threshold-adjustment {
  background: var(--bg-surface);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-md);
}

.threshold-adjustment label {
  display: block;
  color: var(--text-secondary);
  font-weight: 500;
  margin-bottom: var(--spacing-sm);
}

.slider-container {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
}

.slider {
  flex: 1;
  height: 6px;
  border-radius: 3px;
  background: var(--bg-tertiary);
  outline: none;
  -webkit-appearance: none;
}

.slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background: var(--primary-color);
  cursor: pointer;
}

.threshold-value {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--text-primary);
  min-width: 60px;
  text-align: center;
}

.rgb-display {
  display: flex;
  gap: var(--spacing-lg);
  align-items: center;
}

.color-preview {
  width: 100px;
  height: 100px;
  border-radius: var(--border-radius-md);
  border: 2px solid var(--border-color);
  box-shadow: var(--shadow-md);
}

.color-values {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.color-value {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
}

.color-label {
  font-weight: 600;
  color: var(--text-secondary);
  min-width: 20px;
}

.value {
  font-family: 'Monaco', 'Menlo', monospace;
  font-weight: 600;
  color: var(--text-primary);
  background: var(--bg-tertiary);
  padding: var(--spacing-xs) var(--spacing-sm);
  border-radius: var(--border-radius-sm);
  min-width: 50px;
  text-align: center;
}

.color-learning {
  background: var(--bg-surface);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-md);
}

.input-group {
  display: flex;
  gap: var(--spacing-md);
}

.color-input {
  flex: 1;
  padding: var(--spacing-sm) var(--spacing-md);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-md);
  background: var(--bg-tertiary);
  color: var(--text-primary);
  font-size: 0.875rem;
}

.color-input:focus {
  outline: none;
  border-color: var(--primary-color);
  box-shadow: 0 0 0 2px rgba(37, 99, 235, 0.2);
}

.learned-colors {
  background: var(--bg-surface);
  padding: var(--spacing-lg);
  border-radius: var(--border-radius-md);
}

.learned-colors h4 {
  color: var(--text-secondary);
  margin-bottom: var(--spacing-md);
  font-size: 0.875rem;
}

.color-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.color-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-sm);
  background: var(--bg-tertiary);
  border-radius: var(--border-radius-md);
}

.color-swatch {
  width: 30px;
  height: 30px;
  border-radius: var(--border-radius-sm);
  border: 1px solid var(--border-color);
}

.color-name {
  flex: 1;
  color: var(--text-primary);
  font-weight: 500;
}

.profiles-section {
  grid-column: 1 / -1;
}

.profile-controls {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
}

.profile-select {
  padding: var(--spacing-sm) var(--spacing-md);
  border: 1px solid var(--border-color);
  border-radius: var(--border-radius-md);
  background: var(--bg-tertiary);
  color: var(--text-primary);
  font-size: 0.875rem;
}

.profile-info {
  color: var(--text-secondary);
  font-style: italic;
}

.btn-accent {
  background-color: var(--accent-color);
  color: white;
}

.btn-accent:hover:not(:disabled) {
  background-color: #d97706;
}
</style>