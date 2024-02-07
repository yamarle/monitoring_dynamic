#ifndef INITIALIZATION2_HPP
#define INITIALIZATION2_HPP

#include "funciones.hpp"
#include <dirent.h>
#include <cstdio>
#include <sys/stat.h>
//#include <sys/types.h>

#if defined(_WIN32)
	#define OS 0
	#define MKDIR(dir_name) mkdir(dir_name)
#else
	#define OS 1
	#define MKDIR(dir_name) mkdir(dir_name, 0777);
#endif

void clear_DIR(const char* dir_name){
    DIR *dirp = opendir(dir_name);
    if(dirp==NULL){
        // No existe, creo el directorio
        MKDIR(dir_name);
        return;
    }
    struct dirent *dp;

    // Limpiar el directorio (cargarme todo lo que hay dentro)
    //readdir(dirp); readdir(dirp); // ., ..
    dp = readdir(dirp);
    while(dp!=NULL){
		if(string(dp->d_name)[0]!='.') remove((dir_name+string(dp->d_name)).c_str());
        dp = readdir(dirp);
    }
    closedir(dirp);
}

string get_string(string line){
    string res;
    size_t w = line.find(":");
    for(int i=w+1; i<line.size(); i++){
		// espacio en blanco, caracter nulo, salto de línea, carriage return
        if(line[i]!=' ' && line[i]!='\0' && line[i]!='\n' && line[i]!='\r' && line[i]!='\t'){
			res.push_back(line[i]);
		}
    }
    return res;
}

vector<int> get_vector(string line){
    vector<int> res;
    size_t w = line.find(":");
    string num;
    char prev=line[w+1];
    if(prev!=' ') num.push_back(prev);
    for(int i=w+2; i<line.size(); i++){
        if(line[i]!=' ' && line[i]!=',' && line[i]!='\0' && line[i]!='\n' && line[i]!='\r' && line[i]!='\t'){
            num.push_back(line[i]);
            if(i==line.size()-1){
                res.push_back(atoi(num.c_str()));
            }
        }else if((line[i]==' ' || line[i]==',' || line[i]=='\0' || line[i]=='\n' || line[i]=='\r' || line[i]=='\t') && (prev!=' ' && prev!=',' && prev!='\t')){
            res.push_back(atoi(num.c_str()));
            num.clear();
        }
        prev=line[i]; // actualizar el previo
    }
    return res;
}

void initialize(string &scen, string &results, float &vrange, float &crange, float &srange, int &com_type, int &share_info, int &meet_strat, Poss<int> &s, Poss<int> &g, int &iter, int &nseg, int &ngoals, int &res, float &speed, float &time, float &tx_time, float &work_time, int &seed){
    string sep;
    if(OS==1) sep = "/";
    else sep = "\\";
    string filename = ".."+sep+"files"+sep+"config.txt";
    ifstream file(filename);
    string line;
    if(file.is_open()){
            while(getline(file,line)){
                // Iteraciones del algoritmo
                if(line.size()>=10 && line.find("iterations")<=line.size()){
                    iter=atoi(get_string(line).c_str());
                }
                if(line.size()>=10 && line.find("resolution")<=line.size()){
                    res=atoi(get_string(line).c_str());
                }
                // Número de segmentos
                if(line.size()>=9 && line.find("nsegments")<=line.size()){
                    nseg=atoi(get_string(line).c_str());
                }
                // Radio para la comunicación
                if(line.size()>=6 && line.find("crange")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    crange = atof(get_string(line).c_str());
                }
                // Rango de visión
                if(line.size()>=6 && line.find("vrange")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    vrange = atof(get_string(line).c_str());
                }
                // Rango de seguridad
                if(line.size()>=6 && line.find("srange")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    srange = atof(get_string(line).c_str());
                }
                // Tipo de comunicación
                if(line.size()>=8 && line.find("com_type")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    com_type = atof(get_string(line).c_str());
                }
                // Compartir información
                if(line.size()>=10 && line.find("share_info")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    share_info = atof(get_string(line).c_str());
                }
                // Estrategias de encuentro
                if(line.size()>=10 && line.find("meet_strat")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    meet_strat = atof(get_string(line).c_str());
                }
                // Velocidad de los robots
                if(line.size()>=5 && line.find("speed")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    speed = atof(get_string(line).c_str());
                }
                // Número de goals
                if(line.size()>=5 && line.find("ngoals")<=line.size()){
                    ngoals=atoi(get_string(line).c_str());
                }
                // Tiempo de la misión
                if(line.size()>=12 && line.find("mission_time")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    time = atof(get_string(line).c_str());
                }
                // Tiempo para transmitir un objetivo
                if(line.size()>=10 && line.find("trans_time")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    tx_time = atof(get_string(line).c_str());
                }
                // Tiempo de un único trabajo
                if(line.size()>=9 && line.find("work_time")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    work_time = atof(get_string(line).c_str());
                }
                // El fichero del escenario
                if(line.size()>=8 && line.find("scenario")<=line.size()){
                    //scen = line.substr(10);
                    //scen = ".."+sep+"scenarios"+sep+"scen"+line.substr(10)+".txt";
                    scen = ".."+sep+"scenarios"+sep+"scen"+get_string(line)+".txt";
                }
                // La carpeta de los resultados
                if(line.size()>=7 && line.find("results")<=line.size()){
                    results = ".."+sep+"results"+sep+get_string(line)+sep;
                }
                // Semilla para la generación de aleatorios
                if(line.size()>=4 && line.find("seed")<=line.size()){
                    //radio=atof(line.substr(7).c_str());
                    seed = atof(get_string(line).c_str());
                }
                // Posiciones iniciales de los robots
                if(line.size()>=6 && line.find("pos_ini")<=line.size()){
                    // La línea de las "x"
                    getline(file,line);
                    s.x=get_vector(line);
                    // La línea de las "y"
                    getline(file,line);
                    s.y=get_vector(line);
                }
                // Posiciones de los goals
                if(line.size()>=7 && line.find("pos_goal")<=line.size()){
                    // La línea de las "x"
                    getline(file,line);
                    g.x=get_vector(line);
                    // La línea de las "y"
                    getline(file,line);
                    g.y=get_vector(line);
                }
            }
            if(!scen.size()) cerr<<"No se ha indicado el escenario"<<endl;
            if(!results.size()) cerr<<"No se ha indicado la carpeta de los resultados"<<endl;
            else{
                clear_DIR(results.c_str());
            }
            if(!s.x.size()) cerr<<"No se ha indicado la posición inicial de los robots"<<endl;
            if(!g.x.size()) cerr<<"No se ha indicado el objetivo de los robots"<<endl;
            // Cerrar el fichero
            file.close();
    }else cout<<"No se puede abrir este fichero"<<endl;
}

#endif