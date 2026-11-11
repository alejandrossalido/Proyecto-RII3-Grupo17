# Descargar el proyecto

[Volver al README](../README.md)

Necesitas Git instalado. Puedes comprobarlo con:

    git --version

Si no lo tienes, en Ubuntu puedes instalarlo con:

    sudo apt update
    sudo apt install git

Para guardar una copia en tu carpeta Projects:

    mkdir -p ~/Projects
    cd ~/Projects
    git clone https://github.com/alejandrossalido/Sprint-1-PRII.git
    cd Sprint-1-PRII

mkdir crea la carpeta, cd entra en ella y git clone descarga el repositorio. El símbolo ~ es tu carpeta personal. Puedes elegir otra carpeta si lo prefieres.

La copia descargada se llama Sprint-1-PRII y ya contiene src y el README. Esa carpeta es el workspace; hace la misma función que g17_PRII3_ws en mi copia original. No hay que crear otro workspace dentro.

Si usas Distrobox, puedes descargar el proyecto desde la terminal normal, dentro de tu carpeta de usuario, antes de entrar en la caja.

## Actualizar tu copia

Si ya lo tienes descargado, usa estos comandos para traer los últimos cambios:

    cd ~/Projects/Sprint-1-PRII
    git pull --ff-only

## Después de descargar

Clonar descarga el código. Para ejecutarlo también necesitas ROS y las herramientas de la [guía de Ubuntu](ubuntu.md) o de [Distrobox](distrobox.md).

Las carpetas build, install y log se crean al compilar en tu ordenador.
