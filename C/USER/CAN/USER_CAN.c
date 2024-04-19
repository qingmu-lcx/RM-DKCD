#include "USER_CAN.h"

#define Yaw_Position 2000


//判断CAN1接收到的信息来自于底盘电机
#define can1_chassis_motor(id)  ( id == 0x201 || \
                                  id == 0x202 || \
                                  id == 0x203 || \
                                  id == 0x204 )

//判断CAN1接收到的是来自CAN1的yaw轴电机的数据
#define can1_gimbal_motor(id) 	( id == 0x205 || \
                                  id == 0x206 || \
                                  id == 0x207 || \
                                  id == 0x208 )

//判断CAN2接收到的信息来自于底盘电机
#define can2_chassis_motor(id)  ( id == 0x201 || \
                                  id == 0x202 || \
                                  id == 0x203 || \
                                  id == 0x204 )

//判断CAN2接收到的是来自CAN2的yaw轴电机的数据
#define can2_gimbal_motor(id) 	( id == 0x205 || \
                                  id == 0x206 || \
                                  id == 0x207 || \
                                  id == 0x208 )

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static uint8_t CAN1_TX_200[8]={0};//定义CAN1 发送数组1
static uint8_t CAN1_TX_1FF[8]={0};//定义CAN1 发送数组2

static uint8_t CAN2_TX_200[8]={0};//定义CAN2 发送数组1
static uint8_t CAN2_TX_1FF[8]={0};//定义CAN2 发送数组2

MotorBack CAN1_Motor[CAN1_Motor_NUM];	//CAN1上的所有电机回馈信息

MotorBack CAN2_Motor[CAN2_Motor_NUM];	//CAN2上的所有电机回馈信息

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * @brief          初始化CAN总线过滤器
  * @retval         none
  */
void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//32位获取字节
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;	//过滤器0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//选择邮箱FIFO 0
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;//分配筛选器
    can_filter_st.FilterBank = 14;					//过滤器14
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//=================================================================================//
/**
  * @brief          给CAN1电机发生电流数据
	* @param[in]			id: 发送电流值的ID  0x200控制1~4号电机，0x1FF控制5~8号电机
  * @retval         none
  */
//=================================================================================//
void CAN1_to_Motor(uint16_t id)
{
	uint32_t TxMailbox; //发送邮箱
	CAN_TxHeaderTypeDef     tx_message;//定义发送结构体

	tx_message.StdId = id;//  标识符ID
	tx_message.IDE   = CAN_ID_STD;
	tx_message.RTR   = CAN_RTR_DATA;
	tx_message.DLC   = 0x08;

	switch(id)
	{
		case 0x200:
		{
			HAL_CAN_AddTxMessage(&hcan1,&tx_message,CAN1_TX_200,&TxMailbox);//发送
			break;
		}
		case 0x222:
		{
			HAL_CAN_AddTxMessage(&hcan1,&tx_message,CAN1_TX_200,&TxMailbox);//发送
			break;
		}
		case 0x1FF:
		{
			HAL_CAN_AddTxMessage(&hcan1,&tx_message,CAN1_TX_1FF,&TxMailbox);//发送
			break;
		}
		default: break;
	}
}

//=================================================================================//
/**
  * @brief          给CAN2电机发生电流数据
	* @param[in]			id: 发送电流值的ID  0x200控制1~4号电机，0x1FF控制5~8号电机
  * @retval         none
  */
//=================================================================================//
void CAN2_to_Motor(uint16_t id)
{
	uint32_t TxMailbox; //发送邮箱
	CAN_TxHeaderTypeDef     tx_message;//定义发送结构体

	tx_message.StdId = id;//  标识符ID
	tx_message.IDE   = CAN_ID_STD;
	tx_message.RTR   = CAN_RTR_DATA;
	tx_message.DLC   = 0x08;

	switch(id)
	{
		case 0x200:
		{
			HAL_CAN_AddTxMessage(&hcan2,&tx_message,CAN2_TX_200,&TxMailbox);//发送
			break;
		}
		case 0x1FF:
		{
			HAL_CAN_AddTxMessage(&hcan2,&tx_message,CAN2_TX_1FF,&TxMailbox);//发送
			break;
		}
		default: break;
	}
}

//=================================================================================//
/**
  * @brief          修改CAN1上的某个电机的电流值
	* @param[in]			id: 需要修改电流值的电机ID  范围1~8
  * @retval         none
  */
//=================================================================================//
void CAN1_OPerate_Motor(uint8_t id,short i)
{
	switch(id)
	{
		case 1:
		{
			CAN1_TX_200[0] = (uint8_t)(i>>8);
			CAN1_TX_200[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 2:
		{
			CAN1_TX_200[2] = (uint8_t)(i>>8);
			CAN1_TX_200[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 3:
		{
			CAN1_TX_200[4] = (uint8_t)(i>>8);
			CAN1_TX_200[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 4:
		{
			CAN1_TX_200[6] = (uint8_t)(i>>8);
			CAN1_TX_200[7] = (uint8_t)(i&0x00ff);
			break;
		}
		case 5:
		{
			CAN1_TX_1FF[0] = (uint8_t)(i>>8);
			CAN1_TX_1FF[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 6:
		{
			CAN1_TX_1FF[2] = (uint8_t)(i>>8);
			CAN1_TX_1FF[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 7:
		{
			CAN1_TX_1FF[4] = (uint8_t)(i>>8);
			CAN1_TX_1FF[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 8:
		{
			CAN1_TX_1FF[6] = (uint8_t)(i>>8);
			CAN1_TX_1FF[7] = (uint8_t)(i&0x00ff);
			break;
		}
		default: break;
	}
	
}

//=================================================================================//
/**
  * @brief          修改CAN2上的某个电机的电流值
	* @param[in]			id: 需要修改电流值的电机ID  范围1~8
  * @retval         none
  */
//=================================================================================//
void CAN2_OPerate_Motor(uint8_t id,short i)
{
	switch(id)
	{
		case 1:
		{
			CAN2_TX_200[0] = (uint8_t)(i>>8);
			CAN2_TX_200[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 2:
		{
			CAN2_TX_200[2] = (uint8_t)(i>>8);
			CAN2_TX_200[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 3:
		{
			CAN2_TX_200[4] = (uint8_t)(i>>8);
			CAN2_TX_200[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 4:
		{
			CAN2_TX_200[6] = (uint8_t)(i>>8);
			CAN2_TX_200[7] = (uint8_t)(i&0x00ff);
			break;
		}
		case 5:
		{
			CAN2_TX_1FF[0] = (uint8_t)(i>>8);
			CAN2_TX_1FF[1] = (uint8_t)(i&0x00ff);
			break;
		}
		case 6:
		{
			CAN2_TX_1FF[2] = (uint8_t)(i>>8);
			CAN2_TX_1FF[3] = (uint8_t)(i&0x00ff);
			break;
		}
		case 7:
		{
			CAN2_TX_1FF[4] = (uint8_t)(i>>8);
			CAN2_TX_1FF[5] = (uint8_t)(i&0x00ff);
			break;
		}
		case 8:
		{
			CAN2_TX_1FF[6] = (uint8_t)(i>>8);
			CAN2_TX_1FF[7] = (uint8_t)(i&0x00ff);
			break;
		}
		default: break;
	}
}

//=================================================================================//
/**
  * @brief          CAN接收中断回调函数
  * @param[in]      hcan: 产生中断的CAN
  * @retval         none
  */
//=================================================================================//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接收回调函数
{
	CAN_RxHeaderTypeDef		rx_message;//定义接收结构体
	uint8_t rx_data[8];
	uint8_t id = 0;
	if(hcan ==&hcan1)//回调CAN1 RX
	{	    
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_message, rx_data);//获取接收完成标志    
		//在这里解析can1接收数据
		id = (uint8_t)(rx_message.StdId - 0x201);
		if(can1_chassis_motor(rx_message.StdId))	//接收底盘电机数据
		{
			CAN1_Motor[id].Last_Speed = CAN1_Motor[id].Speed;
			CAN1_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN1_Motor[id].Last_Angle = CAN1_Motor[id].Angle;
			CAN1_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN1_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN1_Motor[id].Temperate = rx_data[6];         			
		}
		else if(can1_gimbal_motor(rx_message.StdId))	//接收云台电机数据
		{
             
			CAN1_Motor[id].Last_Speed = CAN1_Motor[id].Speed;
			CAN1_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN1_Motor[id].Last_Angle = CAN1_Motor[id].Angle;
			CAN1_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN1_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN1_Motor[id].Temperate = rx_data[6];
            
                                        /*diversity_Angle 最小回正角*/
			if(Yaw_Position-CAN1_Motor[id].Angle>-4100&&Yaw_Position-CAN1_Motor[id].Angle<2300)//-2500是6100-8192=-2092 
			{
				CAN1_Motor[id].diversity_Angle=Yaw_Position-CAN1_Motor[id].Angle;
			}
			else
			{
				CAN1_Motor[id].diversity_Angle=Yaw_Position+8192-CAN1_Motor[id].Angle;
			} 
								 /*Angle_Radian 弧度角*/
			CAN1_Motor[id].Angle_Radian=Yaw_Position-CAN1_Motor[id].Angle>=0?
			Yaw_Position-CAN1_Motor[id].Angle:8192+Yaw_Position-CAN1_Motor[id].Angle;//3436+8192   //8912
									/*Circle圈数计算·*/
			if((CAN1_Motor[id].Angle-CAN1_Motor[id].Last_Angle)>4100)
				CAN1_Motor[id].Circle--;
			else if((CAN1_Motor[id].Angle-CAN1_Motor[id].Last_Angle)<-4100)
				CAN1_Motor[id].Circle++;

			//所以当两个角度的差值大于4100的时候，肯定是前一次角度为8000左右，下一次角度为200左右，差值就大于4100，而这个时候也刚好转满一圈。
							/*连续角改变Continuous_Angle And Angle_Difference初始角度和预定角度差值*/
			CAN1_Motor[id].Continuous_Angle=(int)CAN1_Motor[id].Circle*8192+
																			CAN1_Motor[id].Angle-CAN1_Motor[id].Last_Continuous_Angle;    //连续角，就是总共走了多少的角度，全部角度之和，与圈数和当前角度还有初始角度有关。
					
			if(CAN1_Motor[id].Flag==0)    //整个程序只执行一次，用于判断电机的初始值
			{
				CAN1_Motor[id].Flag=1;
				CAN1_Motor[id].Last_Continuous_Angle=CAN1_Motor[id].Continuous_Angle;     //这里的"Last_Continuous_Angle"相当于一个初值，就是最开始的时候的初始角度
				CAN1_Motor[id].Angle_Difference=(Yaw_Position-CAN1_Motor[id].Angle);//
			}            
		}
		else
		{
//      Receive_Data(&rx_message,rx_data);
			CAN1_Motor[id].Last_Angle = CAN1_Motor[id].Angle;
			CAN1_Motor[id].Last_Speed = CAN1_Motor[id].Speed;
			CAN1_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN1_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN1_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN1_Motor[id].Temperate = rx_data[6];
		}
		//需要编写中断标志位清零
		__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_IER_FMPIE0);							
	}
	else//回调CAN2 RX
	{	    
		if ( HAL_OK == HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_message, rx_data))//获取接收完成标志
		{
			//在这里解析can2接收数据
			id = (uint8_t)(rx_message.StdId - 0x201);
			CAN2_Motor[id].Last_Speed = CAN2_Motor[id].Speed;
			CAN2_Motor[id].Speed = (short)(rx_data[2]<<8 | rx_data[3]);
			CAN2_Motor[id].Last_Angle = CAN2_Motor[id].Angle;
			CAN2_Motor[id].Angle = (short)(rx_data[0]<<8 | rx_data[1]);
			CAN2_Motor[id].Given_Current = (short)(rx_data[4]<<8 | rx_data[5]);
			CAN2_Motor[id].Temperate = rx_data[6];			
			//需要编写中断标志位清零
			__HAL_CAN_CLEAR_FLAG(&hcan2,CAN_IER_FMPIE0);
		}	 								
	}
}

