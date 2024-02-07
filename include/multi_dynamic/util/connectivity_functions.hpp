
#ifndef CONNECTIVITY_FUNCTIONS_HPP
#define CONNECTIVITY_FUNCTIONS_HPP

#include "funciones.hpp"
#include "bresenham.hpp"

namespace connectivity_functions{

    bool check_connectivity(int x1, int y1, int x2, int y2, vector<vector<float>> com_map, int crange, int com_type)
    {
        if(com_type == 0){
            if(sqrt(pow(x1-x2,2)+pow(y1-y2,2))<=crange) return true;
        }else{
            if(sqrt(pow(x1-x2,2)+pow(y1-y2,2))<=crange && !bresenham::exists_wall(com_map, x1, y1, x2, y2)) return true;
        }
        return false;
    }

    vector<vector<bool>> check_connectivity(Poss<int> pos, float crange)
    {
        int n = pos.x.size();
        vector<vector<bool>> res(n, vector<bool>(n,false));
        for(int i=0; i<n; i++){
            for(int j=0; j<n; j++){
                if(i!=j && sqrt(pow(pos.x[i]-pos.x[j],2)+pow(pos.y[i]-pos.y[j],2))<=crange)
                    res[i][j] = true;
            }
        }

        return res;
    } // Comprobar los enlaces de comunicación entre los agentes, SIN LoS

    vector<vector<bool>> check_connectivity(Poss<int> pos, float crange, vector<vector<float>> real_map)
    {
        int n = pos.x.size();
        vector<vector<bool>> res(n, vector<bool>(n,false));
        for(int i=0; i<n; i++){
            for(int j=0; j<n; j++){
                if(i!=j && !bresenham::exists_wall(real_map, pos.x[i], pos.y[i], pos.x[j], pos.y[j]) && sqrt(pow(pos.x[i]-pos.x[j],2)+pow(pos.y[i]-pos.y[j],2))<=crange)
                    res[i][j] = true;
            }
        }

        return res;
    } // Comprobar los enlaces de comunicación entre los agentes, CON LoS

    vector<vector<bool>> check_connectivity(Poss<int> pos, float crange, vector<vector<bool>> cond_con)
    {
        int n = pos.x.size();
        vector<vector<bool>> res(n, vector<bool>(n,false));
        for(int i=0; i<n; i++){
            for(int j=0; j<n; j++){
                if(cond_con[i][j] && i!=j && sqrt(pow(pos.x[i]-pos.x[j],2)+pow(pos.y[i]-pos.y[j],2))<=crange)
                    res[i][j] = true;
            }
        }

        return res;
    } // Comprobar los enlaces de comunicación entre los agentes, SIN LoS (comunicación condicional)

    vector<vector<bool>> check_connectivity(Poss<int> pos, float crange, vector<vector<float>> real_map, vector<vector<bool>> cond_con)
    {
        int n = pos.x.size();
        vector<vector<bool>> res(n, vector<bool>(n,false));
        for(int i=0; i<n; i++){
            for(int j=0; j<n; j++){
                if(cond_con[i][j] && i!=j && !bresenham::exists_wall(real_map, pos.x[i], pos.y[i], pos.x[j], pos.y[j]) && sqrt(pow(pos.x[i]-pos.x[j],2)+pow(pos.y[i]-pos.y[j],2))<=crange)
                    res[i][j] = true;
            }
        }

        return res;
    } // Comprobar los enlaces de comunicación entre los agentes, CON LoS (comunicación condicional)

    vector<int> agents_groups(vector<vector<bool>> links)
    {
        int n = links.size();
        int no = n;
        vector<int> res(n,-1);

        int ng = 0; // Cantidad de grupos
        res[0] = ng;
        n--;

        queue<int> q; q.push(0);
        while(n){
            for(int i=0; i<no; i++){
                if(res[i]<0){ // No pertenece a ningún grupo
                    if(links[q.front()][i]){
                        q.push(i);
                        res[i] = ng;
                        n--; // Uno más clasificado
                    }
                }
            }
            q.pop(); // quitar de la lista
            if(n && q.empty()){ // Quedan por clasificar, pero son de otro grupo
                for(int i=0; i<no; i++)
                    if(res[i]<0){ // No pertenece a ningún grupo
                        q.push(i);
                        ng++; // Es otro grupo
                        res[i] = ng; // Clasificar
                        n--; // Uno más clasificado
                        break;
                    }
            }
        }

        return res;
    } // Grupos de agentes conectados entre sí (A partir de una matriz de enlaces)

    vector<int> agents_groups(Poss<int> pos, float crange)
    {
        int n = pos.x.size();
        int no = n;
        vector<int> res(n,-1);

        int ng = 0; // Cantidad de grupos

        res[0] = ng;

        queue<int> q; q.push(0);
        while(n){
            for(int i=0; i<no; i++){
                if(res[i]<0){ // No pertenece a ningún grupo
                    if(sqrt(pow(pos.x[q.front()]-pos.x[i],2)+pow(pos.y[q.front()]-pos.y[i],2))<=crange){
                        q.push(i);
                        res[i] = ng;
                        n--;
                    }
                }
            }
            q.pop(); // quitar de la lista
            if(n && q.empty()){
                for(int i=0; i<no; i++)
                    if(res[i]<0){ // No pertenece a ningún grupo
                        q.push(i);
                        ng++;
                        res[i] = ng;
                        n--;
                        break;
                    }
            }
        }

        return res;
    } // Grupos de agentes conectados entre sí (SIN tener en cuenta LoS)

    vector<int> agents_groups(Poss<int> pos, float crange, vector<vector<float>> real_map)
    {
        int n = pos.x.size();
        int no = n;
        vector<int> res(n,-1);

        int ng = 0; // Cantidad de grupos

        res[0] = ng;

        queue<int> q; q.push(0);
        while(n){
            for(int i=0; i<no; i++){
                if(res[i]<0){ // No pertenece a ningún grupo
                    if(!bresenham::exists_wall(real_map, pos.x[i], pos.y[i], pos.x[q.front()], pos.y[q.front()]) && sqrt(pow(pos.x[q.front()]-pos.x[i],2)+pow(pos.y[q.front()]-pos.y[i],2))<=crange){
                        q.push(i);
                        res[i] = ng;
                        n--;
                    }
                }
            }
            q.pop(); // quitar de la lista
            if(n && q.empty()){
                for(int i=0; i<no; i++)
                    if(res[i]<0){ // No pertenece a ningún grupo
                        q.push(i);
                        ng++;
                        res[i] = ng;
                        n--;
                        break;
                    }
            }
        }

        return res;
    } // Grupos de agentes conectados entre sí (TENIENDO en cuenta LoS)

    // Funciones inútiles

    vector<Poss<int>> generate_links(Poss<int> pos, vector<vector<bool>> links)
    {
        vector<Poss<int>> res;   
        for(int i=0; i<pos.x.size(); i++)
            for(int j=i+1; j<pos.x.size(); j++)
                if(links[i][j]){
                    res.resize(res.size()+1);
                    res[res.size()-1].x.push_back(pos.x[i]); res[res.size()-1].y.push_back(pos.y[i]);
                    res[res.size()-1].x.push_back(pos.x[j]); res[res.size()-1].y.push_back(pos.y[j]);
                }
        return res;
    } // ENlaces de comunicación entre los agentes

    vector<Poss<int>> base_links(int x, int y, Poss<int> pos, float crange)
    {
        vector<Poss<int>> res;
        for(int i=0; i<pos.x.size(); i++)
            if(sqrt(pow(x-pos.x[i],2)+pow(y-pos.y[i],2))<=crange){
                res.resize(res.size()+1);
                res[res.size()-1].x.push_back(pos.x[i]); res[res.size()-1].y.push_back(pos.y[i]);
                res[res.size()-1].x.push_back(x); res[res.size()-1].y.push_back(y);
            }
        return res;
    } // Enlaces entre los agentes y la base (NO LoS)

    vector<Poss<int>> base_links(int x, int y, Poss<int> pos, float crange, vector<vector<float>> real_map)
    {
        vector<Poss<int>> res;
        for(int i=0; i<pos.x.size(); i++)
            if(!bresenham::exists_wall(real_map, pos.x[i], pos.y[i], x, y) && sqrt(pow(x-pos.x[i],2)+pow(y-pos.y[i],2))<=crange){
                res.resize(res.size()+1);
                res[res.size()-1].x.push_back(pos.x[i]); res[res.size()-1].y.push_back(pos.y[i]);
                res[res.size()-1].x.push_back(x); res[res.size()-1].y.push_back(y);
            }
        return res;
    } // Enlaces entre los agentes y la base (SI LoS)

    vector<Poss<int>> base_links(int x, int y, Poss<int> pos, float crange, vector<bool> &base_con)
    {
        vector<Poss<int>> res;
        for(int i=0; i<pos.x.size(); i++)
            if(sqrt(pow(x-pos.x[i],2)+pow(y-pos.y[i],2))<=crange){
                base_con[i] = true;
                res.resize(res.size()+1);
                res[res.size()-1].x.push_back(pos.x[i]); res[res.size()-1].y.push_back(pos.y[i]);
                res[res.size()-1].x.push_back(x); res[res.size()-1].y.push_back(y);
            }else{
                base_con[i] = false;
            }
        return res;
    } // Enlaces entre los agentes y la base (NO LoS)

    vector<Poss<int>> base_links(int x, int y, Poss<int> pos, float crange, vector<vector<float>> real_map, vector<bool> &base_con)
    {
        vector<Poss<int>> res;
        for(int i=0; i<pos.x.size(); i++)
            if(!bresenham::exists_wall(real_map, pos.x[i], pos.y[i], x, y) && sqrt(pow(x-pos.x[i],2)+pow(y-pos.y[i],2))<=crange){
                base_con[i] = true;
                res.resize(res.size()+1);
                res[res.size()-1].x.push_back(pos.x[i]); res[res.size()-1].y.push_back(pos.y[i]);
                res[res.size()-1].x.push_back(x); res[res.size()-1].y.push_back(y);
            }else{
                base_con[i] = false;
            }
        return res;
    } // Enlaces entre los agentes y la base (SI LoS)

    // Información completa de la conectividad
    struct connectivity_vars{
        vector<vector<bool>> connectivity;
        vector<bool> base_connectivity;
        vector<Poss<int>> links, blinks; // enlaces entre agentes, incluida la base
        int ngroups = 0; // cantidad de grupos
        vector<int> groups; // índices de a qué grupos pertenece cada agente
    };

    void update_connectivity_vars(connectivity_vars &con_vars, Poss<int> agents, Poss<int> bg, vector<vector<float>> map, int com_type, float crange)
    {
        if(!com_type){
            // Radio
            con_vars.connectivity = check_connectivity(agents, crange);
            //con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange);
            con_vars.base_connectivity.resize(agents.x.size(),false);
            con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange,con_vars.base_connectivity);
        }else{
            // Radio + LoS solo de los obstáculos estáticos o estáticps + dinámicos
            con_vars.connectivity = check_connectivity(agents, crange, map);
            //con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange, map);
            con_vars.base_connectivity.resize(agents.x.size(),false);
            con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange, map, con_vars.base_connectivity);
        }

        // Únicamente para representar
        con_vars.links = generate_links(agents, con_vars.connectivity);
        con_vars.links.insert(con_vars.links.end(), con_vars.blinks.begin(), con_vars.blinks.end());
        
        // Sacar los grupos que forman los agentes
        con_vars.groups = agents_groups(con_vars.connectivity); // Grupos de agentes
        con_vars.ngroups = max_v(con_vars.groups)+1; // Número de grupos
    } // Actualizar las variables de conectividad

    void update_connectivity_vars(connectivity_vars &con_vars, Poss<int> agents, Poss<int> bg, vector<vector<bool>> cond_con, vector<vector<float>> map, int com_type, float crange)
    {
        if(!com_type){
            // Radio
            con_vars.connectivity = check_connectivity(agents, crange, cond_con);
            //con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange);
            con_vars.base_connectivity.resize(agents.x.size(),false);
            con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange,con_vars.base_connectivity);
        }else{
            // Radio + LoS solo de los obstáculos estáticos o estáticps + dinámicos
            con_vars.connectivity = check_connectivity(agents, crange, map, cond_con);
            //con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange, map);
            con_vars.base_connectivity.resize(agents.x.size(),false);
            con_vars.blinks = base_links(bg.x[0],bg.y[0],agents,crange, map, con_vars.base_connectivity);
        }

        // Únicamente para representar
        con_vars.links = generate_links(agents, con_vars.connectivity);
        con_vars.links.insert(con_vars.links.end(), con_vars.blinks.begin(), con_vars.blinks.end());
        
        // Sacar los grupos que forman los agentes
        con_vars.groups = agents_groups(con_vars.connectivity); // Grupos de agentes
        con_vars.ngroups = max_v(con_vars.groups)+1; // Número de grupos
    } // Actualizar las variables de conectividad (Con comuincación condicional)

    void save_links(vector<Poss<int>> links_poss, string filename)
    {
        for(int i=0; i<links_poss.size(); i++){
            save_poss_(filename.c_str(), links_poss[i].x, links_poss[i].y);
        }
    }

}

#endif