/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 21
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 20
#define PRIORITY_TPOS 22
#define PRIORITY_TBATTERY 24
#define ROB 1
#define T100 100000000 //enough clock ticks to do 100 ms 
/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_loadArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_computePos, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_sem_create(&sem_startCam, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_waitResponse, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_cam, "th_cam", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToRobot, "th_sendToRobot", 0, PRIORITY_TBATTERY, 0)) {
    	cerr << "Error task create: " << strerror(-err) << endl << flush;
    	exit(EXIT_FAILURE);
    }

    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;
    if ((err = rt_queue_create(&q_messageToRobot, "q_messageToRobot", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
    	cerr << "Error msg robot queue create: " << strerror(-err) << endl << flush;
    	exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;
   

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cam, (void(*)(void*)) & Tasks::CameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    /*if (err = rt_task_start(&th_pos, (void(*)(void*)) & Tasks::ComputePosition, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }*/
    if (err = rt_task_start(&th_sendToRobot, (void(*)(void*)) & Tasks::WriteToRobot, this)) {
    	cerr << "Error task start: " << strerror(-err) << endl << flush;
    	exit(EXIT_FAILURE);
    }



    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
        	cout << "Connexion avec le moniteur perdue"  << endl << flush;
        	// stop robot
        	rt_mutex_acquire(&mutex_move, TM_INFINITE);
        	move = MESSAGE_ROBOT_STOP;
        	rt_mutex_release(&mutex_move);	 
        	// stop communication with robot and close server
        	this->Stop();   	 
        	// close camera
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                cameraOpen = false;  
                rt_mutex_release(&mutex_camera);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if (msgRcv ->CompareID(MESSAGE_CAM_OPEN))
        {
            rt_sem_v(&sem_startCam);
            
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            cameraOpen = true;  
            rt_mutex_release(&mutex_camera);
           
        }else if (msgRcv ->CompareID(MESSAGE_CAM_CLOSE))
        {
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            cameraOpen = false;  
            rt_mutex_release(&mutex_camera);
            
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ACK));
        }
        else if (msgRcv -> CompareID(MESSAGE_CAM_ASK_ARENA))
        {          
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            arenaDetect = true;  
            rt_mutex_release(&mutex_arena);
        }
        else if (msgRcv -> CompareID(MESSAGE_CAM_ARENA_CONFIRM))
        {
            rt_mutex_acquire(&mutex_loadArena, TM_INFINITE);
            loadedArena = tmpLoadedArena; 
            rt_mutex_release(&mutex_loadArena);
            
            rt_sem_v(&sem_waitResponse);

        }        
        else if (msgRcv -> CompareID(MESSAGE_CAM_ARENA_INFIRM))
        {
            rt_sem_v(&sem_waitResponse);
        }
        else if (msgRcv -> CompareID(MESSAGE_CAM_POSITION_COMPUTE_START))
        {
            rt_mutex_acquire(&mutex_computePos, TM_INFINITE);
            computePos = true;  
            rt_mutex_release(&mutex_computePos);
        }
        else if (msgRcv -> CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP))
        {
            rt_mutex_acquire(&mutex_computePos, TM_INFINITE);
            computePos = false;  
            rt_mutex_release(&mutex_computePos);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}


/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
               
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        
        WriteInQueue(&q_messageToRobot,robot.StartWithoutWD());
        
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, T100);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            WriteInQueue(&q_messageToRobot,new Message((MessageID)cpMove));
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}


 /* Displays battery level*/
void Tasks::BatteryTask(void *arg) {
    int rs;
    int lvl;
    Message* msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /*----------------------------------------------------------------------*/
    /*----------------------THE TASK START HERE ----------------------------*/
    /*----------------------------------------------------------------------*/
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    
    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic battery level check";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs == 1) {
          
            WriteInQueue(&q_messageToRobot,new Message(MESSAGE_ROBOT_BATTERY_GET));
            
            
        }
        
        
        cout << endl << flush;
    }
}

void Tasks::CameraTask(void *arg) {
 
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    //camera open 
    bool co; 
    //arena detect 
    bool ad; 
    //position computing 
    bool cp; 
    Camera cam = Camera();
    cam.Open();
    std::list<Position> robots; 
    Arena a; 
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, T100);

    while (1) {
        rt_sem_p(&sem_startCam, TM_INFINITE);
        cout << "camera open, start taking images" << endl << flush; 
        
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        co = cameraOpen; 
        rt_mutex_release(&mutex_camera);
        
        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
        ad = arenaDetect; 
        rt_mutex_release(&mutex_arena);
        
        rt_mutex_acquire(&mutex_computePos, TM_INFINITE);
        cp = computePos; 
        rt_mutex_release(&mutex_computePos);
        
        while (1) 
        {
            rt_task_wait_period(NULL);
            //if the camera is open and we don't want to detect the arena 
            if (co && !ad)
            {
                cout << "take an image";
                Img img = cam.Grab();
                

                rt_mutex_acquire(&mutex_loadArena, TM_INFINITE);
                a = loadedArena; 
                rt_mutex_release(&mutex_loadArena);
                
                img.DrawArena(a); 
                
                if(cp && !a.IsEmpty())
                {
                    robots = img.SearchRobot(a);
                    Position robot = Position();
                    //find our robot (with id ROB) 
                    for (std::list<Position>::iterator i = robots.begin(); i != robots.end(); i++ ){
                        if (i->robotId == ROB)
                        {
                            robot = *i;
                        }
                    }
                    //if no robot was found it will send a position with the id -1
                    MessagePosition* msgPos = new MessagePosition( MESSAGE_CAM_POSITION, robot);
                    WriteInQueue(&q_messageToMon, msgPos);
                    
                                    
                    if (robot.robotId != -1){
                        //if the robot is detected
                        img.DrawRobot(robot);
                    }   
                }

                Img * p_img = new Img(img);
                MessageImg* msgI = new MessageImg(MESSAGE_CAM_IMAGE, p_img); 
                cout << "send img message to minitor" << endl << flush; 
                WriteInQueue(&q_messageToMon, msgI);
            }
            //if the camera is open and we want to detect the arena 
            if (co && ad)
            {
                cout << "---------------------------------------------" << endl << "arena detection process" << endl << flush; 
                Img img = cam.Grab(); 
                Arena arena = img.SearchArena(); 
                if (arena.IsEmpty()) 
                //if no arena found 
                {
                    cout << "---------------------------------------------" << endl <<  "no arena found" << endl << flush;      
                    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    arenaDetect = false; 
                    rt_mutex_release(&mutex_arena);
                }
                
                else 
                {
                    cout << "arena found" << arena.ToString() << endl << flush; 
                    img.DrawArena(arena); 
                    
                    //load temporally the arena, if the arena is validated then we will save it
                    rt_mutex_acquire(&mutex_loadArena, TM_INFINITE);
                    tmpLoadedArena = arena; 
                    rt_mutex_release(&mutex_loadArena);
                    
                    Img * p_img = new Img(img);
                    MessageImg* msgI = new MessageImg(MESSAGE_CAM_IMAGE, p_img); 
                    cout << "send img message with arena to monitor" << endl << flush; 
                    WriteInQueue(&q_messageToMon, msgI);
                    
                    //wait a response to continue the image sending
                    rt_sem_p(&sem_waitResponse, TM_INFINITE);
                    
                    //end of the arena detection 
                    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    arenaDetect = false; 
                    rt_mutex_release(&mutex_arena);

                  }
                
            }

            else if (!co)
            {
                break; 
            }
            //upload the states of the supervisor
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            co = cameraOpen; 
            rt_mutex_release(&mutex_camera);

            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            ad = arenaDetect; 
            rt_mutex_release(&mutex_arena);
            
            rt_mutex_acquire(&mutex_computePos, TM_INFINITE);
            cp = computePos; 
            rt_mutex_release(&mutex_computePos);
        
        }
    }
}

        
void Tasks::WriteToRobot(void* arg) {
	Message* res;
	Message* msg;
	int compteur=0;
    
	while (1) {
    	//cout << "wait msg to sendRobot" << endl << flush;
    	msg = ReadInQueue(&q_messageToRobot);
    	//cout << "Send msg to robot: " << msg->ToString() << endl << flush;
    	if (compteur < 3) {
       	rt_mutex_acquire(&mutex_robot, TM_INFINITE);
       	res = robot.Write(msg); //msg is destroyed in this
       	rt_mutex_release(&mutex_robot);
      	 
       	if (res->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
            compteur ++;
            if (compteur >= 3) {
               	// previens le serveur
               	Message* msgSend = new Message(MESSAGE_ANSWER_COM_ERROR);
               	WriteInQueue(&q_messageToMon, msgSend);
               	// arrete la comm avec le robot
               	rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
               	robotStarted = 0;
               	rt_mutex_release(&mutex_robotStarted);
               	rt_mutex_acquire(&mutex_robot, TM_INFINITE);

               	robot.Close();
               	rt_mutex_release(&mutex_robot);
               	compteur = 0;

            }
       	}
        else {
            compteur = 0;
            if (res->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            } else if (res->GetID()==MESSAGE_ROBOT_BATTERY_LEVEL){
                WriteInQueue(&q_messageToMon, res);
            }                  	 
        }   	 
    }	 
    }
}

