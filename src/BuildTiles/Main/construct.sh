#! /bin/bash

./start_construct_server.sh

# Start 4 clients
for f in 1 2; do
    ./start_construct_client.sh $f
done