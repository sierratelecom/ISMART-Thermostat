/***************************************************************************//**
* \file PVref_1_PM.c
* \version 1.0
*
* \brief
*  This file provides the API source code for Power Management of the
*  Programmable Voltage Reference component.
*
* Note:
*  None
*
********************************************************************************
* \copyright
* Copyright 2016, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PVref_1.h"

static PVref_1_backup_struct PVref_1_backup = {0u};


/*******************************************************************************
* Function Name: PVref_1_Sleep
****************************************************************************//**
*
*  Prepares the component to enter Sleep mode.
*  If the reference is based on Vdda, the function will open the switch at the
*  bottom of the resistor tree to eliminate the current flow through the
*  resistor. If the reference is based on the bandgap, this function will do
*  nothing.
*
* \sideeffect
*  If the reference is based on Vdda, it will affect all Vdda-based references
*  because they share the same resistor tree.
*
*******************************************************************************/
void PVref_1_Sleep(void)
{
    if (0u != (PVref_1_PRB_REF_REG & PVref_1_VREF_SUPPLY_SEL))
    {
        PVref_1_PRB_CTRL_REG &= ~PVref_1_VDDA_ENABLE;
        PVref_1_backup.enableState = 1u;
    }
    else /* The reference is based on the bandgap */
    {
        PVref_1_backup.enableState = 0u;
    }
}


/*******************************************************************************
* Function Name: PVref_1_Wakeup
****************************************************************************//**
*
*  Restores the component after exiting Sleep mode.
*  By calling this function for any Vdda-based reference, the function will
*  enable all Vdda-based references because they all are based on the same
*  resistor divider. For bandgap-based references, nothing will happen.
*
* \sideeffect
*  If the reference is based on Vdda, it will affect all Vdda-based references
*  because they share the same resistor tree.
*
*******************************************************************************/
void PVref_1_Wakeup(void)
{
    if (0u != PVref_1_backup.enableState)
    {
        PVref_1_PRB_CTRL_REG |= PVref_1_VDDA_ENABLE;
    } /* Do nothing if the reference is based on the bandgap */
}


/* [] END OF FILE */
