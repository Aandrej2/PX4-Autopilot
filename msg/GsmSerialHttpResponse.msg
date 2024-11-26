uint64 timestamp		# time since system start (microseconds)

uint8 type			# type of HTTP request
uint8 GET = 0
uint8 POST = 1

char[512] url			# request url address
char[512] data			# if POST request type, send encoded json data

uint32 response_code		# HTTP Request response code;
