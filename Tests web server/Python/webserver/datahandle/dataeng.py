import sqlite3


class DataEngine:

    def __init__(self, dir, **kwargs):
        self.__directory = dir
        self.__bdd = self.initConn(self.__directory)    # Inicializa la BDD.
        self.__cursor = self.__bdd.cursor()             # Inicializa el cursor.

        # Inicia con modo debug desactivado. El modo debug imprime las consultas en consola.
        self._debugmode = False
        if 'debugmode' in kwargs.keys():  # Si está 'debugmode' como argumento...
            if isinstance(kwargs['debugmode'], bool):  # Si es True o False...
                self._debugmode = kwargs["debugmode"]  # Coloca el valor indicado.
                print('DataEngine._debugmode: ' + str(self._debugmode) )

    def executeCommand(self, query):
        # Query a ejecutar.
        if self._debugmode == True:
            print('>>>QUERY: \n'+str(query))
        self.__cursor.execute(query)
        self.__bdd.commit()

    def readData(self, q):
        # Genera query para leer todas las columnas.
        self.executeCommand(q)
        results = self.__cursor.fetchall()
        if self._debugmode == True:
            print('Query result: '+str(results))
        return results     # Devuelve lo obtenido de la consulta

    def writeData(self, tablename, c, v, **kwargs):
        # Método para escribir datos. Se reciben los datos:
        # tablename -> Nombre de la tabla (String)
        # c -> Lista de columnas (cuyos nombres están en formato String).
        # v -> Lista de conjuntos de valores. Para escribir más de un valor por operación, se deben
        #   mandar más listas dentro de la lista original. todo: Si se recibe una lista sin listas adentro, meterla dentro de otra para que funcione.
        # argumentos: Si en el argumento keyword llamado "force" se coloca True y hay colisión de primary key (1er columna)
        #   en algún lugar de la tabla, éste es sobrescrito. En caso de no poner nada o poner False, no se modifica, pero
        #   se eleva el error. todo: Hacer que avise con un print en el error elevado al sistema.

        c = list(c)     # Castea a lista.
        v = list(v)     # Castea a lista.

        force = False
        if 'force' in kwargs.keys():    # Si está 'force' como argumento...
            if isinstance(kwargs['force'], bool) and len(v) == 1: # Si es True o False, y se modifica SOLO una fila...
                force = kwargs['force']     # Coloca el valor indicado.

        columns = '('      # Empieza con paréntesis.
        for i in c:
            columns += ' {},'.format(i)     # Coloca el nombre de la columna (sin comillas) y una coma entre cada una.
        columns = columns[0:-1] + ')'   # Elimina última coma y agrega paréntesis.

        values = ''
        for valor in v:
            aux = '('  # Empieza con paréntesis.
            for i in valor:
                # print('valor = ' + str(valor))  # DEBUG
                if isinstance(i, str):  # Si el valor es tipo string...
                    i = "'{}'".format(i)    # Le agrega comillas.
                aux += '{},'.format(i)  # Coloca una coma después del mismo.

            aux = aux[0:-1] + '),'  # Elimina última coma y agrega paréntesis.
            # print('   aux = ' + str(aux)) # DEBUG
            values += aux
            # print('   values = ' + str(values))   # DEBUG

        values = values[0:-1]   # Elimina la última coma.
        # print('values = ' + str(values))    # DEBUG

        q = """ INSERT INTO """ + str(tablename) + ' ' + str(columns) + ' ' + """ VALUES
                   """ + str(values) + ';'  # Finaliza el query con ';'
        #   INSERT INTO Table_Name
        #   (Column_1, Column2)
        #   VALUES
        #   (Value1, Value2),
        #   (Value3, Value4)
        #   ... ,
        #   ;

        try:    self.executeCommand(q)  # Ejecuta el query
        except sqlite3.IntegrityError:  # Si ya hay un valor con ese ID...
            if force == True:
                self.updateData(tablename, c, v, (c[0], v[0][0]))  # Hace un update del campo.
            else:
                raise IndexError
                return -1

    def updateData(self, t, c, v, condition):
        if len(v[0]) != len(c): raise ValueError    #Si hay más columnas que valores...

        c = list(c)  # Castea a lista.
        v = list(v[0])  # Castea a lista y toma el único primer valor.

        q = 'UPDATE Pasajeros SET '
        for i in range(0, len(c)):  # Durante el largo de la lista/tupla "c" (para cada columna)
            if isinstance(v[i], str):   v[i] = "'{}'".format(v[i]) #Si es string, agrega comillas.
            q += '{0} = {1}'.format(c[i], v[i]) + ','

        q = q[0:-1]  # Elimina último caracter (',')
        q += ' WHERE {0} = {1};'.format(condition[0], condition[1])  # 'WHERE Column = Value'

        # print (str(q))    #debug
        self.executeCommand(q)
        pass

    def initConn(self, d):     # Conecta a la base de datos
        return sqlite3.connect(d)

    def finishConn(self):   # Desconecta de la base de datos
        self.__bdd.close()
        self.__cursor = 0


def getDbTables(db):
    tupla_tablas = dbResultToList(db.readData("""
                        SELECT name FROM sqlite_master WHERE type = 'table';
                    """))

    tablas = []
    for i in tupla_tablas:
        tablas.append(i[0])

    # print('tablas = '+str(tablas))        # Debug.
    return tablas


def dbResultToList(t):
    # Recibe el resultado de una consulta y lo transforma a una lista de listas.
    aux = []
    for i in t:
        aux.append( list(i) )

    # print(str(aux))   # Debug
    return aux


