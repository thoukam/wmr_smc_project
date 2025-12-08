#!/usr/bin/env bash
set -e

# Définition des chemins
# Chemin vers le dossier racine du workspace (remplacez par le chemin réel si nécessaire)
WORKSPACE_ROOT="$(dirname "$0")/../ros2_ws" 
CONTROLLER_PACKAGE="$WORKSPACE_ROOT/src/wmr_controller"
CONFIG_FILE="$CONTROLLER_PACKAGE/config/wmr_tracking.rviz"

# --- Initialisation de l'environnement ---
source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

# --- Lancement du Programme ---
if [ -f "$CONFIG_FILE" ]; then
    echo "Lancement de RViz2 avec la configuration : $CONFIG_FILE"
    
    # Utilise l'option -d pour charger la configuration
    ros2 run rviz2 rviz2 -d "$CONFIG_FILE"
else
    echo "ERREUR : Le fichier de configuration RViz n'a pas été trouvé à :"
    echo "$CONFIG_FILE"
    echo "Veuillez créer le fichier manuellement en lançant RViz, en configurant 'odom' et les deux 'Path', puis en sauvegardant."
    exit 1
fi