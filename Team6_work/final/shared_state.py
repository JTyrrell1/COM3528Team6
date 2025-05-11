from multiprocessing import Manager

manager = Manager()
shared_state = manager.dict(current_mode="idle", agent_speaking=False)