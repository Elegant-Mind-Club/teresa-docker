# Docker Environment

## Environment Setup
1. Install Docker/Docker Desktop. If you're on MacOS, OrbStack is pretty goated

2. Start up the container
- If the container does not currently exist OR you modified `docker-compose.yaml` or `Dockerfile`:
```
docker-compose up -d --build
```
- For subsequent times you access the container
```
docker-compose start
```
- To get out of the container
```
exit
```
- To delete the container
```
docker-compose down
```
- To temporarily pause the container
```
docker-compose stop
```

3. Go into the container
```
docker exec -it teresa_dev_env bash
```