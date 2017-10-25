# Drone
Scripts de commande des moteurs d'un drone.

# Objectif
J'ai entrepris de réaliser ce drone afin de découvrir la plateforme Raspberry Pi 2B et le pilotage de Hardware.

# Performances
Aujourd'hui, je suis capable de :
- Récupérer la vitesse angulaire de mon châssis sur les 3 axes x, y et z grâce à un ADC connecté à ma RPi via un bus I2C.
- Envoyer des consignes de commande du drone via une manette Xbox360 dont je récupère les valeurs codées sur les différents sticks et gâchette à l'aide du driver.
- Utiliser les valeurs récupérées depuis la manette Xbox360 pour envoyer des signaux PWM aux ESC du drone qui pilotent les moteurs.

En résumé, je suis capable de piloter les moteurs de mon drone via une consigne envoyé depuis ma manette Xbox360. De plus, je récupère les valeurs de mon Gyroscope 3axe.
