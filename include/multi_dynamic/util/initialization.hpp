#ifndef INITIALIZATION_HPP
#define INITIALIZATION_HPP

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

void clear_directory(string dir_name){
    clear_DIR(dir_name.c_str());
}

void create_directory(string dir_name){
    clear_DIR(dir_name.c_str());
}

vector<string> get_directory_files(const char* dir_name)
{
    vector<string> res;
    DIR *dirp = opendir(dir_name);
    if(dirp==NULL){
        // No existe, termino aquí
        return res;
    }
    struct dirent *dp;
    // Recorrer el directorio almacenando los nombres de los ficheros
    
    dp = readdir(dirp);
    while(dp!=NULL){
        //if(string(dp->d_name)[0]!='.') remove((dir_name+string(dp->d_name)).c_str());
        res.push_back(dp->d_name);
        dp = readdir(dirp);
    }
    closedir(dirp);

    return res;
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

struct load_var{
    string sep; // Separador
    string filename; // Nombre del fichero completo (directorio)
    void load()
    {
        if(OS==1) sep = "/";
        else sep = "\\";
        filename = ".."+sep+"files"+sep+"config.txt";
    }
};

namespace initialization{

    //*****************************************************************
    // ESCTRUCTURAS DE DATOS
    //*****************************************************************

    struct scenario_vars{
        string scenario_file;
        string results_folder = "tmp";
        int resolution = 1;

        void show()
        {
            cout<<"SCENARIO VARIABLES:"<<endl;
            cout<<"scenario_file : "<<scenario_file<<endl;
            cout<<"resolution    : "<<resolution<<endl;
            cout<<"results_folder: "<<results_folder<<endl;
        }

        // CARGAR
        void load()
        {
            load_var lv; lv.load(); // Generar el directorio del fichero de configuración

            ifstream file(lv.filename);
            string line;
            if(file.is_open()){ // Abrir fichero
                while(getline(file,line)){ // Leer línea
                    if(line.size()>=10 && line.find("resolution")<=line.size()){
                        resolution=atoi(get_string(line).c_str());
                    }
                    if(line.size()>=8 && line.find("scenario")<=line.size()){
                        scenario_file = ".."+lv.sep+"scenarios"+lv.sep+"scen"+get_string(line)+".txt";
                    }
                    if(line.size()>=7 && line.find("results")<=line.size()){
                        results_folder = ".."+lv.sep+"results"+lv.sep+get_string(line)+lv.sep;
                    }
                }

                if(!scenario_file.size()) cerr<<"No se ha indicado el escenario"<<endl;
                if(!results_folder.size()) cerr<<"No se ha indicado la carpeta de los resultados"<<endl;
                else{
                    clear_DIR(results_folder.c_str());
                }

                file.close();
            }else cout<<"No se puede abrir el fichero de configuración"<<endl;
        }

    }; // Variables del escenario (de donde cargo y donde guardo)

    struct agent_settings{
        float vrange = 10; // Rango de visión
        float crange = 10; // Rango de comunicación
        float srange = 0; // Rango de seguridad
        float speed = 1; // Velocidad máxima alcanzable por el agente

        void show()
        {
            cout<<"AGENT PARAMETERS:"<<endl;
            cout<<"vrange: "<<vrange<<endl;
            cout<<"crange: "<<crange<<endl;
            cout<<"srange: "<<srange<<endl;
            cout<<"speed : "<<speed<<endl;
        }

        // CARGAR
        void load()
        {
            load_var lv; lv.load(); // Generar el directorio del fichero de configuración

            ifstream file(lv.filename);
            string line;
            if(file.is_open()){ // Abrir fichero
                while(getline(file,line)){ // Leer línea
                    if(line.size()>=6 && line.find("crange")<=line.size()){
                        crange = atof(get_string(line).c_str());
                    }
                    if(line.size()>=6 && line.find("vrange")<=line.size()){
                        vrange = atof(get_string(line).c_str());
                    }
                    if(line.size()>=6 && line.find("srange")<=line.size()){
                        srange = atof(get_string(line).c_str());
                    }
                    if(line.size()>=5 && line.find("speed")<=line.size()){
                        speed = atof(get_string(line).c_str());
                    }
                }
                file.close();
            }else cout<<"No se puede abrir el fichero de configuración"<<endl;
        }

    }; // Parámetros propios del agente (rangos, velocidad...)

    struct global_param{
        float mission_time = 100;
        int iterations = 100;
        int seed = 0; // Semilla para la generación de valores aleatorios
        Poss<int> pos_ini; // Posición de inicio (referencia)
        Poss<int> pos_goal; // Posición del objetivo

        void show()
        {
            cout<<"GLOBAL PARAMETERS:"<<endl;
            cout<<"mission_time: "<<mission_time<<endl;
            cout<<"iterations  : "<<iterations<<endl;
            cout<<"seed        : "<<seed<<endl;
            cout<<"pos_ini     : ("<<pos_ini.x[0]<<", "<<pos_ini.y[0]<<endl;
        }

        // CARGAR
        void load()
        {
            load_var lv; lv.load(); // Generar el directorio del fichero de configuración

            ifstream file(lv.filename);
            string line;
            if(file.is_open()){ // Abrir fichero
                while(getline(file,line)){ // Leer línea
                    if(line.size()>=12 && line.find("mission_time")<=line.size()){
                        mission_time = atof(get_string(line).c_str());
                    }
                    if(line.size()>=10 && line.find("iterations")<=line.size()){
                        iterations=atoi(get_string(line).c_str());
                    }
                    if(line.size()>=4 && line.find("seed")<=line.size()){
                        seed = atoi(get_string(line).c_str());
                    }
                    // Posiciones iniciales de los robots
                    if(line.size()>=6 && line.find("pos_ini")<=line.size()){
                        // La línea de las "x"
                        getline(file,line);
                        pos_ini.x=get_vector(line);
                        // La línea de las "y"
                        getline(file,line);
                        pos_ini.y=get_vector(line);
                    }
                    // Posiciones de los goals
                    if(line.size()>=7 && line.find("pos_goal")<=line.size()){
                        // La línea de las "x"
                        getline(file,line);
                        pos_goal.x=get_vector(line);
                        // La línea de las "y"
                        getline(file,line);
                        pos_goal.y=get_vector(line);
                    }
                }
                file.close();
            }else cout<<"No se puede abrir el fichero de configuración"<<endl;
        }

    }; // Parámetros para cualquier escenario

    struct mission_param{
        int nagents = 10;
        int ngoals = 10;
        float tx_time = 1; // Tiempo de transmisión
        float work_time = 3; // Tiempo de trabajo (monitor, gather...)
        int com_type = 0; // Tipo de comunicación entre los agentes (0: dist, 1: dist+LoS estáticos, 2: dist+LoS todos)
        int share_info = 1; // Si los agentes comparten información entre ellos o no (PODRÍA SER UN BOOL)
        int meet_strat = 2; // Estrategía de encuentro (LAS 4 ESTRATEGÍAS, ESTO ES PROPIO DE LAS MISSIONES DE RECONEXIÓN)

        void show()
        {
            cout<<"MISSION PARAMETERS:"<<endl;
            cout<<"nagents  : "<<nagents<<endl;
            cout<<"ngoals   : "<<ngoals<<endl;
            cout<<"tx_time  : "<<tx_time<<endl;
            cout<<"work_time: "<<work_time<<endl;
        }

        // CARGAR
        void load()
        {
            load_var lv; lv.load(); // Generar el directorio del fichero de configuración

            ifstream file(lv.filename);
            string line;
            if(file.is_open()){ // Abrir fichero
                while(getline(file,line)){ // Leer línea
                    if(line.size()>=7 && line.find("nagents")<=line.size()){
                        nagents=atoi(get_string(line).c_str());
                    }
                    if(line.size()>=5 && line.find("ngoals")<=line.size()){
                        ngoals=atoi(get_string(line).c_str());
                    }
                    if(line.size()>=7 && line.find("tx_time")<=line.size()){
                        tx_time = atof(get_string(line).c_str());
                    }
                    if(line.size()>=9 && line.find("work_time")<=line.size()){
                        work_time = atof(get_string(line).c_str());
                    }
                    if(line.size()>=8 && line.find("com_type")<=line.size()){
                        com_type = atoi(get_string(line).c_str());
                    }
                    if(line.size()>=10 && line.find("share_info")<=line.size()){
                        share_info = atoi(get_string(line).c_str());
                    }
                    if(line.size()>=10 && line.find("meet_strat")<=line.size()){
                        meet_strat = atoi(get_string(line).c_str());
                    }
                }
                file.close();
            }else cout<<"No se puede abrir el fichero de configuración"<<endl;
        }

    }; // Parámetros propios de la misón

    struct data_gather{
        int ncollectors = 0;
        int nworkers = 0;
        int nsegments = 0;
        int segmentation_type = 1; // PAP por defecto

        void show()
        {
            cout<<"DATA GATHERING PARAMETERS:"<<endl;
            cout<<"ncollectors : "<<ncollectors<<endl;
            cout<<"nworkers    : "<<nworkers<<endl;
            cout<<"nsegments   : "<<nsegments<<endl;
            cout<<"segment_type: "<<segmentation_type<<endl;
        }

        // CARGAR
        void load()
        {
            load_var lv; lv.load(); // Generar el directorio del fichero de configuración

            ifstream file(lv.filename);
            string line;
            if(file.is_open()){ // Abrir fichero
                while(getline(file,line)){ // Leer línea
                    if(line.size()>=11 && line.find("ncollectors")<=line.size()){
                        ncollectors=atoi(get_string(line).c_str());
                    }
                    if(line.size()>=8 && line.find("nworkers")<=line.size()){
                        nworkers=atoi(get_string(line).c_str());
                    }
                    if(line.size()>=9 && line.find("nsegments")<=line.size()){
                        nsegments = atoi(get_string(line).c_str());
                    }
                    if(line.size()>=17 && line.find("segmentation_type")<=line.size()){
                        segmentation_type = atoi(get_string(line).c_str());
                    }
                }
                file.close();
            }else cout<<"No se puede abrir el fichero de configuración"<<endl;
        }

    }; // Parámetros propios de las misiones de data gathering

}

#endif
