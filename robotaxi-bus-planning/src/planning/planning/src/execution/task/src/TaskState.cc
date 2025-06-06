/*
 * this file was generated by NunniFSMGen - do not edit!
 */


#include "src/execution/task/include/TaskState.h"
#include "src/execution/task/include/TaskFSM.h"
namespace acu{
namespace planning{





TaskIdleTaskState * TaskIdleTaskState::m_instance = 0;


TaskIdleTaskState::TaskIdleTaskState() {}


TaskIdleTaskState::~TaskIdleTaskState() {}


TaskIdleTaskState * TaskIdleTaskState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskIdleTaskState;
    return m_instance;
}


void TaskIdleTaskState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->Idle2AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskAnalyseTaskState::instance() );
}


void TaskIdleTaskState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskIdleTaskState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


TaskAnalyseTaskState * TaskAnalyseTaskState::m_instance = 0;


TaskAnalyseTaskState::TaskAnalyseTaskState() {}


TaskAnalyseTaskState::~TaskAnalyseTaskState() {}


TaskAnalyseTaskState * TaskAnalyseTaskState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskAnalyseTaskState;
    return m_instance;
}


void TaskAnalyseTaskState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->Idle2AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskExecuteTaskState::instance() );
}


void TaskAnalyseTaskState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskAnalyseTaskState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskAnalyseTaskState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskAnalyseTaskState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskAnalyseTaskState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


TaskExecuteTaskState * TaskExecuteTaskState::m_instance = 0;


TaskExecuteTaskState::TaskExecuteTaskState() {}


TaskExecuteTaskState::~TaskExecuteTaskState() {}


TaskExecuteTaskState * TaskExecuteTaskState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskExecuteTaskState;
    return m_instance;
}


void TaskExecuteTaskState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskAnalyseTaskState::instance() );
}


void TaskExecuteTaskState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskExceptionalTaskState::instance() );
}


void TaskExecuteTaskState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskExecuteTaskState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskSuspendTaskState::instance() );
}


void TaskExecuteTaskState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskExceptionalTaskState::instance() );
}


void TaskExecuteTaskState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEmergencyBtnHandleState::instance() );
}


void TaskExecuteTaskState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskRemoteControlState::instance() );
}


void TaskExecuteTaskState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExecuteTaskState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExecuteTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


TaskSuspendTaskState * TaskSuspendTaskState::m_instance = 0;


TaskSuspendTaskState::TaskSuspendTaskState() {}


TaskSuspendTaskState::~TaskSuspendTaskState() {}


TaskSuspendTaskState * TaskSuspendTaskState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskSuspendTaskState;
    return m_instance;
}


void TaskSuspendTaskState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskSuspendTaskState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->Suspend2AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskAnalyseTaskState::instance() );
}


void TaskSuspendTaskState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskSuspendTaskState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->SuspendTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


TaskExceptionalTaskState * TaskExceptionalTaskState::m_instance = 0;


TaskExceptionalTaskState::TaskExceptionalTaskState() {}


TaskExceptionalTaskState::~TaskExceptionalTaskState() {}


TaskExceptionalTaskState * TaskExceptionalTaskState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskExceptionalTaskState;
    return m_instance;
}


void TaskExceptionalTaskState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskExceptionalTaskState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->Suspend2AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskAnalyseTaskState::instance() );
}


void TaskExceptionalTaskState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskExceptionalTaskState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->ExceptionalTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


TaskEmergencyBtnHandleState * TaskEmergencyBtnHandleState::m_instance = 0;


TaskEmergencyBtnHandleState::TaskEmergencyBtnHandleState() {}


TaskEmergencyBtnHandleState::~TaskEmergencyBtnHandleState() {}


TaskEmergencyBtnHandleState * TaskEmergencyBtnHandleState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskEmergencyBtnHandleState;
    return m_instance;
}


void TaskEmergencyBtnHandleState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskEmergencyBtnHandleState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->Suspend2AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskAnalyseTaskState::instance() );
}


void TaskEmergencyBtnHandleState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEmergencyBtnHandleState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EmergencyBtnHandleFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


TaskRemoteControlState * TaskRemoteControlState::m_instance = 0;


TaskRemoteControlState::TaskRemoteControlState() {}


TaskRemoteControlState::~TaskRemoteControlState() {}


TaskRemoteControlState * TaskRemoteControlState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskRemoteControlState;
    return m_instance;
}


void TaskRemoteControlState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskEndTaskState::instance() );
}


void TaskRemoteControlState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskRemoteControlState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->Remote2AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskAnalyseTaskState::instance() );
}


void TaskRemoteControlState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->RemoteControlFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


TaskEndTaskState * TaskEndTaskState::m_instance = 0;


TaskEndTaskState::TaskEndTaskState() {}


TaskEndTaskState::~TaskEndTaskState() {}


TaskEndTaskState * TaskEndTaskState::instance() {
    if ( m_instance == 0 )
        m_instance = new TaskEndTaskState;
    return m_instance;
}


void TaskEndTaskState::TaskIsNew( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->Idle2AnalyseTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskAnalyseTaskState::instance() );
}


void TaskEndTaskState::TaskExecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::TaskUnexecutable( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::TaskExecuteOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::TaskExecuteFault( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::TaskOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::TaskAborted( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::TaskSuspended( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::TaskContinued( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::SyetemException( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::SyetemNormal( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::BtnEmergencyPressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::BtnEmergencyUnpressed( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::RemoteCtrlConnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::RemoteCtrlUnconnected( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->EndTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
}


void TaskEndTaskState::EndTaskHandleOver( TaskFSM *ctx,  void * o ) throw (TaskLogicError) {
    try {
        ctx->IdleTaskFunc( o );
    }
    catch( TaskLogicError &e ) {
        ctx->changeState( TaskExceptionalTaskState::instance() );
        throw;
    }
        ctx->changeState( TaskIdleTaskState::instance() );
}
}
}

