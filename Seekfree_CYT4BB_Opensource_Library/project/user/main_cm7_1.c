/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_1
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "../code/app/remote/remote_debug.h"
#include "../code/hmi/ui/ui_manager.h"

#define WIFI_CORE0_READY_MAGIC 0x57494649u

#pragma location = 0x28006C00
__no_init volatile uint32 g_wifi_core0_ready;

// CM7_1 runs the remote debug UI.
// Runtime state can be filled from CM7_0 through shared memory/IPC later.
static void ui_core1_task(void)
{
    Ctrl_Input_t ctrl = {0};
    Nav_Input_t nav_input = {0};
    Nav_Output_t nav_output = {0};
    Nav_State_t nav_state = {0};
    Vision_Result_t vision = {0};
    Vision_Mode_t vision_mode = VISION_MODE_LINE;

    ui_init(UI_PAGE_REMOTE);

    while (true) {
        ui_update(&ctrl, &nav_input, &nav_output, &nav_state, &vision, vision_mode);
        system_delay_ms(10);
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_info_init();

    while (g_wifi_core0_ready != WIFI_CORE0_READY_MAGIC) {
        SCB_InvalidateDCache_by_Addr((uint32 *)&g_wifi_core0_ready, 32u);
        system_delay_ms(1);
    }
    zf_log(0, "CM7_1 booted.");
    zf_log(0, "CM7_1 saw core0 ready.");

    remote_debug_init();
    interrupt_global_enable(0);

    ui_core1_task();

    return 0;
}

// **************************** �������� ****************************
