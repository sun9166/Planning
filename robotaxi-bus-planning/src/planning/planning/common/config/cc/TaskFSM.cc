/*
 * this file was generated by NunniFSMGen - do not edit!
 */


#include "TaskFSM.h"
#include "TaskState.h"
namespace acu{
namespace planning{



TaskFSM::TaskFSM() {}


TaskFSM::TaskFSM( void *o ) : Task( o ) {
    m_state = TaskIdleTaskState::instance();
}


TaskFSM::~TaskFSM() {}


void TaskFSM::TaskIsNew( void * o ) throw (LogicError) {
    m_state->TaskIsNew( this, o );
}


void TaskFSM::TaskExecutable( void * o ) throw (LogicError) {
    m_state->TaskExecutable( this, o );
}


void TaskFSM::TaskUnexecutable( void * o ) throw (LogicError) {
    m_state->TaskUnexecutable( this, o );
}


void TaskFSM::TaskExecuteOver( void * o ) throw (LogicError) {
    m_state->TaskExecuteOver( this, o );
}


void TaskFSM::TaskExecuteFault( void * o ) throw (LogicError) {
    m_state->TaskExecuteFault( this, o );
}


void TaskFSM::TaskOver( void * o ) throw (LogicError) {
    m_state->TaskOver( this, o );
}


void TaskFSM::TaskAborted( void * o ) throw (LogicError) {
    m_state->TaskAborted( this, o );
}


void TaskFSM::TaskSuspended( void * o ) throw (LogicError) {
    m_state->TaskSuspended( this, o );
}


void TaskFSM::TaskContinued( void * o ) throw (LogicError) {
    m_state->TaskContinued( this, o );
}


void TaskFSM::SyetemException( void * o ) throw (LogicError) {
    m_state->SyetemException( this, o );
}


void TaskFSM::SyetemNormal( void * o ) throw (LogicError) {
    m_state->SyetemNormal( this, o );
}


void TaskFSM::BtnEmergencyPressed( void * o ) throw (LogicError) {
    m_state->BtnEmergencyPressed( this, o );
}


void TaskFSM::BtnEmergencyUnpressed( void * o ) throw (LogicError) {
    m_state->BtnEmergencyUnpressed( this, o );
}


void TaskFSM::RemoteCtrlConnected( void * o ) throw (LogicError) {
    m_state->RemoteCtrlConnected( this, o );
}


void TaskFSM::RemoteCtrlUnconnected( void * o ) throw (LogicError) {
    m_state->RemoteCtrlUnconnected( this, o );
}


void TaskFSM::EndTaskHandleOver( void * o ) throw (LogicError) {
    m_state->EndTaskHandleOver( this, o );
}


void TaskFSM::changeState( TaskState *newState ) {
    m_state = newState;
}
}
}

