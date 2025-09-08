import { createApp } from 'vue'
import App from './App.vue'
import './style.css'

// Import des services ROS
import ROSService from './services/ROSService.js'

const app = createApp(App)

// Initialiser le service ROS
app.config.globalProperties.$ros = ROSService

app.mount('#app')