# Drone
Scripts de commande des moteurs d'un drone.

# Objectif
J'ai entrepris de réaliser ce drone afin de découvrir la plateforme Raspberry Pi 2B et le pilotage de Hardware.

# Performances
Aujourd'hui, je suis capable de :
- Récupérer la vitesse angulaire de mon châssis sur les 3 axes x, y et z grâce à un gyroscope analogique 3 axes et un ADC connecté à ma RPi via un bus I2C.
- Envoyer des consignes de commande du drone via une manette Xbox360 dont je récupère les valeurs codées sur les différents sticks et gâchettes à l'aide du driver.
- Utiliser les valeurs récupérées depuis la manette Xbox360 pour envoyer des signaux PWM aux ESC du drone qui pilotent les moteurs.

En résumé, je suis capable de piloter les moteurs de mon drone via une consigne envoyé depuis ma manette Xbox360. De plus, je récupère les valeurs de mon Gyroscope 3 axes.

# Fichiers
- ABE_ADCPi : Driver de l'ADC
- ABE_Helpers : Driver du bus I2C
- drone : script de commande des moteurs (Envoi des signaux PWM aux ESC en fonction des consignes)
- xbox_read : script de récupération des valeurs de consignes codées sur les sticks et gâchettes de la manette Xbox 360 à l'aide du driver.

# Lien
- Video : https://www.youtube.com/watch?v=UWEQvqE_tSw
