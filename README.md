# Robot_Projeto1

## No PC

Conectar-se ao wifi asimov3.



Conectar-se à raspberry:

Você precisa do IPBerry e da senha (t0rt1g4). O IPBerry deve ser visto na tela do seu robô.

    ssh pi@$IPBerry
    
Os comandos da seção a seguir devem ser digitados na Raspberry Pi (via ssh).

Certifique-se de que o cabo da bateria (plug J2 vermelho) está conectado.

Ligue o equipamento na chave de liga-desliga


## Rode na Raspberry:

Primeiramente precisamos do gerenciador de sessões, para fazer caber vários terminais num só ssh:

    screen
  
Atalhos do screen:
Cria uma sub-sessão:`Ctrl A, C`
Mudar de sub-sessão:`Ctrl A, "` usar `shift + '` para fazer o `"`

**(1)** Rode na primeira sub-sessão para conectar a Raspberry a placa de cima do robo.
  
    roslaunch turtlebot3_bringup turtlebot3_core.launch

**(2)** Rode na segunda sub-sessão para transmitir os dados do laser para o computador.

    roslaunch turtlebot3_bringup turtlebot3_lidar.launch

**(3)** Rode na terceira sub-sessão para iniciar o serviço de stream da câmera.

    roslaunch raspicam_node camerav2_640x480_30fps.launch

## No terminal bash do seu computador

Rode `sudo code ~/.bashrc` e verifique que:

**(1)**`export IPBerry=`IP que aparece no robo.

**(2)**	`export ROS_MASTER_URI="http://"$IPBerry":11311"`
        `export ROS_IP=hostname -I`
        `export TURTLEBOT3_MODEL=burger`
        Estão no seu arquivo ~/.bashrc
        
**(3)** Agora execute:

    roscore

**(4)** Para abrir o turtlebot:

    rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz

**(5)** Sempre que a câmera estiver invertida, rode para ajustá-la:

    rosrun rqt_reconfigure rqt_reconfigure

**(6)** Para acionar o comando por teclas:

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
        
## No robô novamente, para desligar

Conecte-se ao ssh

Digite:

	sudo shutdown -P now

Assim que as luzes da Raspberry pararem de piscar desligue a energia no cabo
