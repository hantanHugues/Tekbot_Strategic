const { contextBridge, ipcRenderer } = require('electron');

// Exposer des APIs protégées au contexte de rendu (renderer process)
contextBridge.exposeInMainWorld('electronAPI', {
  // IPC pour la communication avec le processus principal
  send: (channel, data) => {
    // Liste blanche des canaux autorisés
    const validChannels = ['app-quit', 'window-minimize', 'window-maximize'];
    if (validChannels.includes(channel)) {
      ipcRenderer.send(channel, data);
    }
  },
  
  receive: (channel, func) => {
    const validChannels = ['app-version', 'system-status'];
    if (validChannels.includes(channel)) {
      // Délibérément supprimer le premier argument de event
      ipcRenderer.on(channel, (event, ...args) => func(...args));
    }
  },

  // API pour les opérations système
  getVersion: () => ipcRenderer.invoke('get-version'),
  
  // API pour ROS (sera utilisée par roslibjs)
  platform: process.platform
});