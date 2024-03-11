rm -rf log/*




CONFIG_FILE="../config/base.yml"

# Parse the YAML file to get the list of port numbers
ports=$(cat "$CONFIG_FILE" | grep -Eo '^[a-zA-Z]+_port:[ ]+[0-9]+' | awk -F'[: ]+' '{print $NF}')

# Loop through the ports and kill the corresponding processes
for port in $ports; do
    pids=$(lsof -ti :$port)
    if [ -n "$pids" ]; then
        echo "Killing processes using port $port"
        kill -9 $pids
    else
        echo "No processes found using port $port"
    fi
done
