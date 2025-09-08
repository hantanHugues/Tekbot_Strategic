<template>
  <div id="app" class="app-container">
    <!-- Sidebar Navigation -->
    <nav class="sidebar">
      <div class="sidebar-header">
        <div class="logo-container">
          <div class="logo-icon">🤖</div>
          <h1 class="logo-text">TEKBOT</h1>
        </div>
        <div class="subtitle">Control Center</div>
      </div>

      <div class="nav-menu">
        <div 
          v-for="item in navigationItems" 
          :key="item.id"
          class="nav-item"
          :class="{ active: currentView === item.id }"
          @click="setCurrentView(item.id)"
        >
          <span class="nav-icon">{{ item.icon }}</span>
          <span class="nav-label">{{ item.label }}</span>
          <div class="nav-indicator" v-if="currentView === item.id"></div>
        </div>
      </div>

      <!-- Connection Status -->
      <div class="connection-status">
        <div class="status-header">État ROS2</div>
        <div class="status-item" :class="rosConnectionStatus.class">
          <div class="status-dot animate-pulse" v-if="rosConnectionStatus.class === 'status-connected'"></div>
          <div class="status-dot" v-else></div>
          <span>{{ rosConnectionStatus.text }}</span>
        </div>
      </div>
    </nav>

    <!-- Main Content Area -->
    <main class="main-content">
      <!-- Header Bar -->
      <header class="header-bar">
        <div class="header-left">
          <h2 class="page-title">{{ currentPageTitle }}</h2>
        </div>
        <div class="header-right">
          <div class="header-time">{{ currentTime }}</div>
          <div class="header-status">
            <div class="status-indicator" :class="systemStatus.class">
              <div class="status-dot"></div>
              {{ systemStatus.text }}
            </div>
          </div>
        </div>
      </header>

      <!-- Dynamic Content -->
      <div class="content-area">
        <component :is="currentComponent" />
      </div>
    </main>
  </div>
</template>

<script>
import { ref, computed, onMounted, onUnmounted } from 'vue'

// Import des composants
import DashboardMatch from './components/DashboardMatch.vue'
import CalibrationModule from './components/CalibrationModule.vue'
import MissionPlanning from './components/MissionPlanning.vue'
import SystemHealth from './components/SystemHealth.vue'

export default {
  name: 'App',
  components: {
    DashboardMatch,
    CalibrationModule,
    MissionPlanning,
    SystemHealth
  },
  setup() {
    const currentView = ref('dashboard')
    const currentTime = ref('')
    let timeInterval = null

    const navigationItems = [
      { id: 'dashboard', label: 'Dashboard Match', icon: '🎯' },
      { id: 'planning', label: 'Planification', icon: '🗺️' },
      { id: 'calibration', label: 'Calibration', icon: '⚙️' },
      { id: 'health', label: 'System Health', icon: '💚' }
    ]

    const rosConnectionStatus = ref({
      class: 'status-disconnected',
      text: 'Déconnecté'
    })

    const systemStatus = ref({
      class: 'status-warning',
      text: 'Initialisation...'
    })

    const currentComponent = computed(() => {
      const components = {
        'dashboard': 'DashboardMatch',
        'planning': 'MissionPlanning',
        'calibration': 'CalibrationModule',
        'health': 'SystemHealth'
      }
      return components[currentView.value]
    })

    const currentPageTitle = computed(() => {
      const titles = {
        'dashboard': 'Dashboard de Match',
        'planning': 'Planification de Mission',
        'calibration': 'Calibration des Capteurs',
        'health': 'État du Système'
      }
      return titles[currentView.value]
    })

    const setCurrentView = (viewId) => {
      currentView.value = viewId
    }

    const updateTime = () => {
      const now = new Date()
      currentTime.value = now.toLocaleTimeString('fr-FR', {
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
      })
    }

    const initializeROS = () => {
      // Simuler la connexion ROS pour l'instant
      setTimeout(() => {
        rosConnectionStatus.value = {
          class: 'status-connected',
          text: 'Connecté'
        }
        systemStatus.value = {
          class: 'status-connected',
          text: 'Système Opérationnel'
        }
      }, 2000)
    }

    onMounted(() => {
      updateTime()
      timeInterval = setInterval(updateTime, 1000)
      initializeROS()
    })

    onUnmounted(() => {
      if (timeInterval) {
        clearInterval(timeInterval)
      }
    })

    return {
      currentView,
      currentTime,
      navigationItems,
      rosConnectionStatus,
      systemStatus,
      currentComponent,
      currentPageTitle,
      setCurrentView
    }
  }
}
</script>

<style scoped>
.app-container {
  display: flex;
  height: 100vh;
  background-color: var(--bg-primary);
}

/* Sidebar Styles */
.sidebar {
  width: 280px;
  background: linear-gradient(180deg, var(--bg-secondary) 0%, var(--bg-tertiary) 100%);
  border-right: 1px solid var(--border-color);
  display: flex;
  flex-direction: column;
  position: relative;
}

.sidebar::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: linear-gradient(45deg, rgba(37, 99, 235, 0.1) 0%, rgba(16, 185, 129, 0.05) 100%);
  pointer-events: none;
}

.sidebar-header {
  padding: var(--spacing-xl);
  border-bottom: 1px solid var(--border-color);
  position: relative;
  z-index: 1;
}

.logo-container {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  margin-bottom: var(--spacing-sm);
}

.logo-icon {
  font-size: 2rem;
  background: linear-gradient(135deg, var(--primary-color), var(--secondary-color));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.logo-text {
  font-size: 1.5rem;
  font-weight: 700;
  background: linear-gradient(135deg, var(--text-primary), var(--text-secondary));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.subtitle {
  color: var(--text-muted);
  font-size: 0.875rem;
  margin-left: 3rem;
}

.nav-menu {
  flex: 1;
  padding: var(--spacing-lg);
  position: relative;
  z-index: 1;
}

.nav-item {
  display: flex;
  align-items: center;
  padding: var(--spacing-md) var(--spacing-lg);
  margin-bottom: var(--spacing-sm);
  border-radius: var(--border-radius-lg);
  cursor: pointer;
  transition: all 0.3s ease;
  position: relative;
  background: rgba(255, 255, 255, 0.02);
}

.nav-item:hover {
  background: rgba(255, 255, 255, 0.08);
  transform: translateX(4px);
}

.nav-item.active {
  background: linear-gradient(135deg, rgba(37, 99, 235, 0.2), rgba(16, 185, 129, 0.1));
  box-shadow: 0 4px 15px rgba(37, 99, 235, 0.2);
}

.nav-icon {
  font-size: 1.25rem;
  margin-right: var(--spacing-md);
}

.nav-label {
  font-weight: 500;
  color: var(--text-secondary);
}

.nav-item.active .nav-label {
  color: var(--text-primary);
}

.nav-indicator {
  position: absolute;
  right: 0;
  top: 50%;
  transform: translateY(-50%);
  width: 3px;
  height: 60%;
  background: linear-gradient(180deg, var(--primary-color), var(--secondary-color));
  border-radius: 2px;
}

.connection-status {
  padding: var(--spacing-lg);
  border-top: 1px solid var(--border-color);
  position: relative;
  z-index: 1;
}

.status-header {
  font-size: 0.75rem;
  font-weight: 600;
  text-transform: uppercase;
  color: var(--text-muted);
  margin-bottom: var(--spacing-sm);
}

.status-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  padding: var(--spacing-sm);
  border-radius: var(--border-radius-md);
  font-size: 0.875rem;
  font-weight: 500;
}

/* Main Content Styles */
.main-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  background-color: var(--bg-primary);
}

.header-bar {
  height: 80px;
  background: var(--bg-secondary);
  border-bottom: 1px solid var(--border-color);
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 var(--spacing-2xl);
  box-shadow: var(--shadow-sm);
}

.page-title {
  font-size: 1.5rem;
  font-weight: 600;
  color: var(--text-primary);
}

.header-right {
  display: flex;
  align-items: center;
  gap: var(--spacing-lg);
}

.header-time {
  font-family: 'Monaco', 'Menlo', monospace;
  font-size: 1.125rem;
  font-weight: 600;
  color: var(--text-secondary);
  background: var(--bg-tertiary);
  padding: var(--spacing-sm) var(--spacing-md);
  border-radius: var(--border-radius-md);
  border: 1px solid var(--border-color);
}

.content-area {
  flex: 1;
  overflow: auto;
  padding: var(--spacing-2xl);
}
</style>