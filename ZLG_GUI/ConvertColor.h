/****************************************************************************************
* 文件名：CONVERTCOLOR.H
* 功能：颜色值转换程序。(头文件)
* 作者：
* 日期：
* 备注：
****************************************************************************************/
#ifndef  CONVERTCOLOR_H
#define  CONVERTCOLOR_H



/****************************************************************************
* 名称：GUI_Color2Index_565()
* 功能：将RGB值转换16位索引值。转换后的值适用于64K色彩色液晶。
* 入口参数：ColorRGB		RGB值，d23--d16为R值，d15--d8为G值，d7--d0为B值
* 出口参数：返回转换值(64K色，d15--d11为R值，d10--d5为G值，d4--d0为B值)
* 说明：
****************************************************************************/
extern  uint16  GUI_Color2Index_565(uint32 colorRGB);


/****************************************************************************
* 名称：GUI_Index2Color_565()
* 功能：将16位索引值转换为RGB值。适用于64K色彩色液晶。
* 入口参数：index		16位索引值(64K色，d15--d11为R值，d10--d5为G值，d4--d0为B值)
* 出口参数：返回值即为RGB值(d23--d16为R值，d15--d8为G值，d7--d0为B值)。
* 说明：
****************************************************************************/
extern  uint32  GUI_Index2Color_565(uint16 index);


/****************************************************************************
* 名称：GUI_Color2Index_555()
* 功能：将RGB值转换15位索引值。转换后的值适用于32K色彩色液晶。
* 入口参数：ColorRGB		RGB值，d23--d16为R值，d15--d8为G值，d7--d0为B值
* 出口参数：返回转换值(32K色，d14--d10为R值，d9--d5为G值，d4--d0为B值)
* 说明：
****************************************************************************/
extern  uint16  GUI_Color2Index_555(uint32 colorRGB);


/****************************************************************************
* 名称：GUI_Index2Color_555()
* 功能：将15位索引值转换为RGB值。适用于32K色彩色液晶。
* 入口参数：index		15位索引值(32K色，d14--d10为R值，d9--d5为G值，d4--d0为B值)
* 出口参数：返回值即为RGB值(d23--d16为R值，d15--d8为G值，d7--d0为B值)。
* 说明：
****************************************************************************/
extern  uint32  GUI_Index2Color_555(uint16 index);


/****************************************************************************
* 名称：GUI_Color2Index_444()
* 功能：将RGB值转换12位索引值。转换后的值适用于4096色伪彩液晶。
* 入口参数：ColorRGB		RGB值，d23--d16为R值，d15--d8为G值，d7--d0为B值
* 出口参数：返回转换值(4096色，RRRRGGGGBBBB)
* 说明：
****************************************************************************/
extern  uint16  GUI_Color2Index_444(uint32 colorRGB);



/****************************************************************************
* 名称：GUI_Index2Color_444()
* 功能：将12位索引值转换为RGB值。适用于4096色伪彩液晶。
* 入口参数：index		12位索引值(4096色，RRRRGGGGBBBB)
* 出口参数：返回值即为RGB值(d23--d16为R值，d15--d8为G值，d7--d0为B值)。
* 说明：
****************************************************************************/
extern  uint32  GUI_Index2Color_444(uint16 index);



/****************************************************************************
* 名称：GUI_Color2Index_332()
* 功能：将RGB值转换8位索引值。转换后的值适用于256色伪彩液晶。
* 入口参数：ColorRGB		RGB值，d23--d16为R值，d15--d8为G值，d7--d0为B值
* 出口参数：返回转换值(256色，RRRGGGBB)
* 说明：
****************************************************************************/
extern  uint8  GUI_Color2Index_332(uint32 colorRGB);




/****************************************************************************
* 名称：GUI_Index2Color_332()
* 功能：将8位索引值转换为RGB值。适用于256色伪彩液晶。
* 入口参数：index		8位索引值(256色，RRRGGGBB)
* 出口参数：返回值即为RGB值(d23--d16为R值，d15--d8为G值，d7--d0为B值)。
* 说明：
****************************************************************************/
extern  uint32  GUI_Index2Color_233(uint8 index);


/****************************************************************************
* 名称：GUI_Color2Index_222()
* 功能：将RGB值转换8位索引值。转换后的值适用于64色液晶。
* 入口参数：ColorRGB		RGB值，d23--d16为R值，d15--d8为G值，d7--d0为B值
* 出口参数：返回转换值(64色，RRGGBB)
* 说明：
****************************************************************************/
extern  uint8  GUI_Color2Index_222(uint32 colorRGB);



/****************************************************************************
* 名称：GUI_Index2Color_222()
* 功能：将6位索引值转换为RGB值。适用于64色液晶。
* 入口参数：index		6位索引值(64色，RRGGBB)
* 出口参数：返回值即为RGB值(d23--d16为R值，d15--d8为G值，d7--d0为B值)。
* 说明：
****************************************************************************/
extern  uint32  GUI_Index2Color_222(uint8 index);



/****************************************************************************
* 名称：GUI_Color2Index_111()
* 功能：将RGB值转换3位索引值。转换后的值适用于8色液晶或8级恢度液晶。
* 入口参数：ColorRGB		RGB值，d23--d16为R值，d15--d8为G值，d7--d0为B值
* 出口参数：返回转换值(8色，RGB)
* 说明：
****************************************************************************/
extern  uint8  GUI_Color2Index_111(uint32 colorRGB);


/****************************************************************************
* 名称： GUI_Index2Color_111()
* 功能：将3位索引值转换为RGB值。适用于8色液晶或8级恢度液晶。
* 入口参数：index		3位索引值(8色，RGB)
* 出口参数：返回值即为RGB值(d23--d16为R值，d15--d8为G值，d7--d0为B值)。
* 说明：
****************************************************************************/
extern  uint32  GUI_Index2Color_111(uint8 Index);


#endif



