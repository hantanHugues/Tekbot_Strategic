const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const isDev = process.env.NODE_ENV === 'development';

function createWindow() {
  // Créer la fenêtre du navigateur
  const mainWindow = new BrowserWindow({
    width: 1600,
    height: 1000,
    minWidth: 1200,
    minHeight: 800,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      enableRemoteModule: false,
      preload: path.join(__dirname, 'preload.js')
    },
    titleBarStyle: 'hiddenInset',
    show: false,
    icon: path.join(__dirname, '../assets/icon.png')
  });

  // Charger l'application
  if (isDev) {
    mainWindow.loadURL('http://localhost:5000');
    // Ouvrir les DevTools en mode développement
    mainWindow.webContents.openDevTools();
  } else {
    mainWindow.loadFile(path.join(__dirname, '../../dist/index.html'));
  }

  // Afficher la fenêtre quand elle est prête
  mainWindow.once('ready-to-show', () => {
    mainWindow.show();
  });

  // Empêcher la navigation vers des sites externes
  mainWindow.webContents.on('will-navigate', (event, navigationUrl) => {
    const parsedUrl = new URL(navigationUrl);
    if (parsedUrl.origin !== 'http://localhost:5000' && !isDev) {
      event.preventDefault();
    }
  });
}

// Cette méthode sera appelée quand Electron aura fini
// de s'initialiser et sera prêt à créer des fenêtres de navigateur
app.whenReady().then(createWindow);

// Quitter quand toutes les fenêtres sont fermées, sauf sur macOS
app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  // Sur macOS, il est courant de recréer une fenêtre dans l'app quand
  // l'icône du dock est cliquée et qu'il n'y a pas d'autres fenêtres ouvertes
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});

// Dans ce fichier, vous pouvez inclure le reste du code de processus principal spécifique à votre app
// Vous pouvez aussi le mettre dans des fichiers séparés et les importer ici