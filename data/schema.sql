PRAGMA journal_mode=WAL;

CREATE TABLE IF NOT EXISTS Executions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    time TEXT
);

CREATE TABLE IF NOT EXISTS Positions (
    time REAL,
    robot INTEGER,
    x REAL,
    y REAL,
    execution INTEGER,
    FOREIGN KEY (execution) REFERENCES Execution(id)
);

CREATE TABLE IF NOT EXISTS Formation (
    time REAL,
    robot INTEGER,
    ux REAL,
    uy REAL,
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

CREATE TABLE IF NOT EXISTS VisitedPoints (
    time REAL,
    robot INTEGER,
    visitedPoints BLOB,
    raggio REAL,
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

CREATE TABLE FinalMaps(
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