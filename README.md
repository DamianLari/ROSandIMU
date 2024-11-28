# Projet ROS : Client NTRIP avec Centrale SBG Ellipse

Ce projet implémente un client NTRIP en utilisant ROS pour collecter des corrections RTCM et envoyer des trames NMEA à un serveur NTRIP. Le système est basé sur l'utilisation d'une centrale inertielle **SBG Ellipse** et du driver ROS **sbg_ros_driver** pour interfacer la centrale avec ROS.

## Matériel Utilisé
- **Centrale Inertielle** : SBG Ellipse
- **GNSS** : Utilisation du GNSS intégré pour fournir les données de position à partir du topic ROS.

## Packages ROS Utilisés
- **sbg_ros_driver** : Permet de récupérer les données de position GNSS (latitude, longitude, altitude) à partir de la centrale SBG Ellipse via le topic `/sbg/gps_pos`.

## Fonctionnalités du Script
- **Client NTRIP** : Ce script implémente un client NTRIP pour se connecter à un serveur NTRIP et recevoir des corrections RTCM.
- **Trames NMEA GGA** : Génère des trames NMEA GGA à partir des données de position fournies par le driver `sbg_ros_driver` et les envoie au serveur NTRIP.
- **Publication des Données RTCM** : Les corrections RTCM reçues sont publiées sur le topic ROS `/ntrip/rtcm_data`.

## Configuration des Crédentials
Les informations d'authentification du serveur NTRIP (nom d'utilisateur et mot de passe) sont stockées dans un fichier séparé `credentials.txt`. Voici un exemple de fichier :

```txt
username=VotreNomUtilisateur
password=VotreMotDePasse
```



## Prérequis
- **ROS** (Robot Operating System) doit être installé.
- **Python 3**.
- **sbg_ros_driver** doit être installé et configuré pour la centrale SBG Ellipse.

## Paramètres de Connexion NTRIP
- **Serveur** : `78.24.131.136`
- **Port** : `2101`
- **Mountpoint** : `PRS30`

## Auteur
Ce projet a été développé dans le cadre de travaux de recherche utilisant des capteurs GNSS et NTRIP pour améliorer la précision de la localisation.

