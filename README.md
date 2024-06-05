## Python Installation

Its reccomended that you set up a viruall python envirnment. Note, you must use version 3.8 or 3.10 of python.

**Note** it must be 3.8 or 3.10 because thats what RDK currently supports.

Run the following in the root of the project.

```bash
python3.10 -m venv venv
```

The above will set up a virtual envirnment directory. Next you simply activate it.

```bash
source venv/bin/activate
```

This will activate the virtual envirnment. Now all thats left is to install the dependencies

```bash
python -m pip install -r requirements.txt
```

You can run tests like the following

```bash
cd src
DEBUG=rizon:.* python test_debug.py
```

Here is how you can run the robot

```bash
cd src
sudo DEBUG='rizon:.*' ../venv/bin/python main.py --port 3000 --host localhost
```

Note you can optionally pass

```bash
--robot-ip 192.168.0.100 --local-ip 192.168.0.104
```

Also you can try connecting to the deployed instance of robot viewer like this

```bash
cd src
sudo DEBUG='rizon:.*' ../venv/bin/python main.py --url https://robot-viewer.com
```
