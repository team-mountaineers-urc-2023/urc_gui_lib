import urllib3

def can_access_internet():
	http = urllib3.PoolManager()
	try:
		http.request('GET', 'http://1.1.1.1/', timeout=urllib3.Timeout(1))
		return True
	except:
		return False
