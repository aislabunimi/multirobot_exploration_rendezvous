PRAGMA journal_mode=WAL;

CREATE TABLE IF NOT EXISTS Executions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    time TEXT,
    map TEXT,
    robot_nr INTEGER,
    rendezvous INTEGER,
    notes TEXT --aggiungere la colonna metodo
);

CREATE TABLE IF NOT EXISTS Positions (
    time REAL,
    robot INTEGER,
    x REAL,
    y REAL,
    execution INTEGER,
    FOREIGN KEY (execution) REFERENCES Execution(id)
);

CREATE TABLE IF NOT EXISTS Clustering (
    time REAL,
    robot INTEGER,
    max INTEGER,
    map BLOB,
    points BLOB,
    "union" INTEGER,
    execution INTEGER,
    FOREIGN KEY (execution) REFERENCES Execution(id)
);

CREATE TABLE IF NOT EXISTS Exploration (
    time REAL,
    robot INTEGER,
    frontiers BLOB,
    execution INTEGER,
    blob_frontier INTEGER,
    FOREIGN KEY (execution) REFERENCES Execution(id)
);

CREATE TABLE IF NOT EXISTS FinalMaps(
    execution INTEGER,
    robot INTEGER,
    map BLOB,
    time REAL,
    FOREIGN KEY (execution) REFERENCES Execution(id)
);

CREATE TABLE IF NOT EXISTS RawMaps(
    name TEXT,
    map BLOB
);