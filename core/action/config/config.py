def load_api_key():
    with open('api_key.txt') as f:
        key = f.readline()
        return key 
