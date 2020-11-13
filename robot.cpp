//
// Created by azurec on 2020/11/13.
//

#include "robot.h"
bool FLAG_ONLINE = true;

static void handle(int sig){
    if (sig == SIGINT){
        FLAG_ONLINE = false;
        std::cout << "interrupt caught." << std::endl;
    }
}

void *robotcontrol(void *pVoid){
    gSysRunning.m_gWorkStatus = sys_working_POWER_ON;
    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON)
    {
        ActivateMaster();
        ecstate = 0;
        gSysRunning.m_gWorkStatus = sys_working_SAFE_MODE;
        std::cout << "sys_working_SAFE_MODE." << std::endl;
    }

//    ecrt_master_receive(master);

    while (FLAG_ONLINE){
        signal(SIGINT,handle);
        usleep(100000/TASK_FREQUENCY);
        cyclic_task();
    }

    releaseMaster();
    pthread_exit(NULL);
}

void cyclic_task(void){
    static int current_pos = 0;
    int init_pos[2] = {0,0};
    if(gSysRunning.m_gWorkStatus == sys_working_POWER_ON)
        return;

    // when cycle_count >= 90000 reset counting
    static int cycle_count = 0;
    cycle_count++;
    if (cycle_count >= 90*1000){
        cycle_count = 0;
        run = 0;
    }

    ecrt_master_receive(master);
    ecrt_domain_process(domainRx);
    ecrt_domain_process(domainTx);

    check_domain_state();

    if(!(cycle_count % 500)){
        check_master_state();
        check_slave_config_states();
    }

    switch (gSysRunning.m_gWorkStatus) {
        case sys_working_SAFE_MODE:{
            check_master_state();
            check_slave_config_states();

            std::cout << (master_state.al_states & ETHERCAT_STATUS_OP) << std::endl;

            if ((master_state.al_states & ETHERCAT_STATUS_OP))
            {
                bool tmp = true;
                for(int i = 0; i < 2 ; i++){
                    if (sc_state[i].al_state != ETHERCAT_STATUS_OP){
                        tmp = false;
                        break;
                    }
                }

                if (tmp){
                    ecstate = 0;
                    gSysRunning.m_gWorkStatus = sys_working_OP_MODE;
                    std::cout << "sys_working_OP_MODE" << std::endl;
                }


            }
        }break;

        case sys_working_OP_MODE:{
            ecstate++;
            if(ecstate <= 16){
                for (int i = 0; i < 2 ; i++){
                    switch (ecstate) {
                        case 1:
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x80);
                            break;
                        case 7:
                            //TODO: when to set operation mode ? (case-7 or case-9)
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[i], 9);

                            // set velocity = 0 and then obtain actual position.
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[i], 0);
                            init_pos[i] = EC_READ_S32(domainTx_pd + offset.actual_position[i]);
                            break;
                        case 9:
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i],0x06);
                            break;;
                        case 11:
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i],0x07);
                            break;
                        case 13:
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i],0xF);
                            break;
                    }
                }
            }
            else
            {
                int tmp = true;
                for (int i = 0; i<2 ; i++){
                    if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & (STATUS_SERVO_ENABLE_BIT)) == 0){
                        tmp = false;
                        break;
                    }
                }

                if (tmp){
                    ecstate = 0;
                    gSysRunning.m_gWorkStatus = sys_working_IDLE_STATUS;
                    // IDLE_STATUS means NO operation to ctrl word. and then data exchange in default state.
                    std::cout << "sys_working_IDLE_STATUS" << std::endl;
                }
            }
        }break;

        default:
        {
            if (!(cycle_count % 500)){ // show message per 500 ms.
                std::cout << "actual position of slave-1: " << EC_READ_S32(domainTx_pd + offset.actual_position[0]) << std::endl;
                std::cout << "actual position of slave-2: " << EC_READ_S32(domainTx_pd + offset.actual_position[1]) << std::endl;
                std::cout << "actual velocity of slave-1: " << EC_READ_S32(domainTx_pd + offset.actual_velocity[0]) << std::endl;
                std::cout << "actual velocity of slave-2: " << EC_READ_S32(domainTx_pd + offset.actual_velocity[1]) << std::endl;

            }
            current_pos += 100;
            for (int i = 0; i< 2;i++)
                EC_WRITE_S32(domainRx_pd + offset.target_velocity[i],50);
        }break;
    }

    // send process data
    ecrt_domain_queue(domainRx);
    ecrt_domain_queue(domainTx);
    ecrt_master_send(master);
}

