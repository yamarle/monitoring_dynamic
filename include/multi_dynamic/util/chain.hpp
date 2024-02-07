#ifndef CHAIN_HPP
#define CHAIN_HPP

#include "fmm_2.hpp"
#include "path.hpp"
#include "hungarian.hpp"
#include "bresenham.hpp"

// ================================================================================================================
//                                  FUNCIONES PARA LA CADENA
// ================================================================================================================

namespace Chain{

    Path cadena(Poss<int> bg, vector<vector<float>> grid)
    {
        Path res;
        if(!grid[bg.x[0]][bg.y[0]] || !grid[bg.x[1]][bg.y[1]]){
            return res;
        }
        FMM gr(bg.x[0],bg.y[0],grid);
        vector<vector<float>> grad = gr.compute_gradient_();
        res.gradient_descent_(grad, bg.x[1], bg.y[1]);
        return res;
    }

    // CHEQUEO DE LA CADENA:
    // - 0: cadena posible
    // - 1: no hay camino posible
    // - 2: no es alcanzable con los robots de los que se dispone

    Poss<int> relay_pos(Path cadena, float dthres, int &unreachable)
    {
        Poss<int> res;
        if(cadena.tam<=1){
            unreachable = 1;
            return res;
        }
        float d;
        int ind = cadena.tam-1;
        int x, y;
        x = cadena.x[ind]; y = cadena.y[ind];
        res.x.insert(res.x.begin(),x); res.y.insert(res.y.begin(),y);
        for(int i=cadena.tam-2; i; i--){
            d = sqrt(pow(x-cadena.x[i],2)+pow(y-cadena.y[i],2));
            if(d <= dthres){
                ind = i;
            }else{
                x = cadena.x[ind]; y = cadena.y[ind]; // Almacenar la posición del último enlace viable
                res.x.insert(res.x.begin(),x); res.y.insert(res.y.begin(),y);
            }
        }

        //if(sqrt(pow(res.x[0]-cadena.x[0],2)+pow(res.y[0]-cadena.y[0],2))<=dthres) unreachable = 0;
        //else unreachable = 2;

        if(sqrt(pow(res.x[0]-cadena.x[0],2)+pow(res.y[0]-cadena.y[0],2)) > dthres) unreachable = 2;

        return res;
    }

    Poss<int> relay_pos(Path cadena, float dthres, vector<vector<float>> grid, int &unreachable)
    {
        Poss<int> res;
        if(cadena.tam<=1){
            unreachable = 1;
            return res;
        }
        float d;
        int ind = cadena.tam-1;
        int ind_ = ind;
        int x, y;
        x = cadena.x[ind]; y = cadena.y[ind];
        res.x.insert(res.x.begin(),x); res.y.insert(res.y.begin(),y);
        //for(int i=cadena.tam-2; i; i--){
        int i=cadena.tam-2;
        while(i){
            d = sqrt(pow(x-cadena.x[i],2)+pow(y-cadena.y[i],2));
            //if(d <= dthres && !bresenham::exists_wall(grid, x, y, cadena.x[i], cadena.y[i])){
            if(d <= dthres && !bresenham::exists_wall(grid, x, y, cadena.x[i], cadena.y[i])){ // Estoy dentro de rango de comunicación y no hay oclusión
                //ind_ = ind; // Actualizar el punto previo
                ind = i; // Almaceno el índice como punto viable
            }else{ // Ya estoy fuera del rango de comunicación
                if(ind != ind_){ // NO es el mismo punto que tenía almacenado
                    //i = ind_;
                    x = cadena.x[ind]; y = cadena.y[ind]; // Almacenar la posición del último enlace viable
                    res.x.insert(res.x.begin(),x); res.y.insert(res.y.begin(),y); // Almacenar resultado total
                    ind_ = ind; // La posición anterior
                }
            }
            i--;
        }

        //if(sqrt(pow(res.x[0]-cadena.x[0],2)+pow(res.y[0]-cadena.y[0],2))<=dthres && !bresenham::exists_wall(grid, res.x[0], res.y[0], cadena.x[0], cadena.y[0])) unreachable = 0; // Hay un enlace entre la base y el primer agente de la cadena
        //else unreachable = 2;

        if(sqrt(pow(res.x[0]-cadena.x[0],2)+pow(res.y[0]-cadena.y[0],2)) > dthres || bresenham::exists_wall(grid, res.x[0], res.y[0], cadena.x[0], cadena.y[0]))
            unreachable = 2; // NO hay un enlace entre la base y el primer agente de la cadena

        return res;
    }

    struct Chain{
        Path path; // camino sobre el que se posicionarán los agentes
        Poss<int> pos; // Posiciones de relays que deberán alcanzar los agentes
        int unfeasible = 0; // Variable que almacena si es posible el despliegue de la cadena("0"), si no hay 2 motivos ("1": no hay camino o el último relay no alcanza la base, "2": no hay suficientes agentes)
    }; // Objeto cadena

    // =================================================================================================================================

    Poss<int> find_con_pos(Path path, Poss<int> pos, float dthres)
    {
        // El camino va desde la base hasta el goal
        Poss<int> res; res(1);
        // Encontrar el punto
        int it = 0; // EMPIEZO POR LA BASE PARA PODER COMPROBAR FACILMENTE FUERA
        res.x[0] = path.x[it]; res.y[0] = path.y[it];
        while(res.x[0] != pos.x[0] && res.y[0] != pos.y[0]){
            // Si se tiene señal con la base y con el primer relay de la cadena
            if(sqrt(pow(res.x[0]-pos.x[0],2)+pow(res.y[0]-pos.y[0],2)) > dthres && sqrt(pow(res.x[0]-path.x[0],2)+pow(res.y[0]-path.y[0],2)) > dthres){
                break;
            }else{
                it++; // Siguiente punto
                res.x[0] = path.x[it]; res.y[0] = path.y[it];
            }
        }
        return res;
    } // Encontrar la posición (relay) que une el principio del camino (BS) con un punto (pos) CONSIDERNADO LoS

    Poss<int> find_con_pos(Path path, Poss<int> pos, float dthres, vector<vector<float>> grid)
    {
        // El camino va desde la base hasta el goal
        Poss<int> res; res(1);
        // Encontrar el punto
        int it = 0; // EMPIEZO POR LA BASE PARA PODER COMPROBAR FACILMENTE FUERA
        res.x[0] = path.x[it]; res.y[0] = path.y[it];
        while(res.x[0] != pos.x[0] && res.y[0] != pos.y[0]){
            // Si se tiene señal con la base y con el primer relay de la cadena
            if(sqrt(pow(res.x[0]-pos.x[0],2)+pow(res.y[0]-pos.y[0],2)) > dthres && sqrt(pow(res.x[0]-path.x[0],2)+pow(res.y[0]-path.y[0],2)) > dthres && bresenham::exists_wall(grid, res.x[0], res.y[0], pos.x[0], pos.y[0]) && bresenham::exists_wall(grid, res.x[0], res.y[0], path.x[0], path.y[0])){
                break;
            }else{
                it++; // Siguiente punto
                res.x[0] = path.x[it]; res.y[0] = path.y[it];
            }
        }
        return res;
    } // Encontrar la posición (relay) que une el principio del camino (BS) con un punto (pos) CONSIDERNADO LoS

    void check_and_fix_plan(Chain &plan, int nagents, float dthres)
    {
        if(plan.unfeasible == 2){ // No se puede deplegar la cadena, no hay agentes suficientes
            if(nagents - plan.pos.x.size()){ // Quedan agentes
                // Intentar encontrar una última posición
                Poss<int> pos = find_con_pos(plan.path, plan.pos, dthres);
                if(pos.x[0] != plan.path.x[0] && pos.y[0] != plan.path.y[0]){ // No es la base
                    // Inserto el punto
                    plan.pos.x.insert(plan.pos.x.begin(),pos.x[0]);
                    plan.pos.y.insert(plan.pos.y.begin(),pos.y[0]);
                    // Ahora el plan es viable
                    plan.unfeasible = 0;
                }
            }
        }
    } // Comprobar si se puede "apañar" lo del último enlace (sin considerar LoS)

    void check_and_fix_plan(Chain &plan, int nagents, float dthres, vector<vector<float>> com_map)
    {
        if(plan.unfeasible == 2){ // No se puede deplegar la cadena, no hay agentes suficientes
            if(nagents - plan.pos.x.size()){ // Quedan agentes
                // Intentar encontrar una última posición
                Poss<int> pos = find_con_pos(plan.path, plan.pos, dthres, com_map);
                if(pos.x[0] != plan.path.x[0] && pos.y[0] != plan.path.y[0]){ // No es la base
                    // Inserto el punto
                    plan.pos.x.insert(plan.pos.x.begin(),pos.x[0]);
                    plan.pos.y.insert(plan.pos.y.begin(),pos.y[0]);
                    // Ahora el plan es viable
                    plan.unfeasible = 0;
                }
            }
        }
    } // Comprobar si se puede "apañar" lo del último enlace (considerando LoS)

    // =================================================================================================================================

    Chain compute_chain_plan(Poss<int> sg, int com_type, float crange, int nag, vector<vector<float>> grid, vector<vector<float>> com_map)
    {
        Chain res;

        // Camino para extender el camino
        res.path = cadena(sg, grid);

        // Posiciones de relays
        if(!com_type){ // Solo el rando de comunicación
            res.pos = relay_pos(res.path, crange, res.unfeasible);
        }else if(com_type == 1){ // Rango de comunicación + oclusiones obstáculos estáticos
            res.pos = relay_pos(res.path, crange, grid, res.unfeasible);
            check_and_fix_plan(res, nag, crange, grid);
        }else if(com_type == 2){ // Rango de comunicación + oclusiones obstáculos estáticos y dinámicos
            res.pos = relay_pos(res.path, crange, com_map, res.unfeasible);
            check_and_fix_plan(res, nag, crange, com_map);
        }

        if(res.pos.x.size() > nag){ // No hay agentes suficientes para desplegar los relays
            res.unfeasible = 2;
        }

        return res;
    } // calcular el plan de la cadena (Calculando el camino)

    Chain compute_chain_plan(Path path, int com_type, float crange, int nag, vector<vector<float>> grid, vector<vector<float>> com_map)
    {
        Chain res;

        // Camino para extender el camino
        res.path = path;

        // Posiciones de relays
        if(!com_type){ // Solo el rando de comunicación
            res.pos = relay_pos(res.path, crange, res.unfeasible);
        }else if(com_type == 1){ // Rango de comunicación + oclusiones obstáculos estáticos
            res.pos = relay_pos(res.path, crange, grid, res.unfeasible);
            check_and_fix_plan(res, nag, crange, grid);
        }else if(com_type == 2){ // Rango de comunicación + oclusiones obstáculos estáticos y dinámicos
            res.pos = relay_pos(res.path, crange, com_map, res.unfeasible);
            check_and_fix_plan(res, nag, crange, com_map);
        }

        if(res.pos.x.size() > nag){ // No hay agentes suficientes para desplegar los relays
            res.unfeasible = 2;
        }

        return res;
    } // calcular el plan de la cadena (Pasandole ya el camino)

    void save_chain_plan(Chain plan, string base_name)
    {
        plan.path.save((base_name + "_path.txt").c_str());
        plan.pos.save(base_name + "_pos.txt");
        save_vect((base_name + "_unfeasible.txt").c_str(),vector<int>(1,plan.unfeasible));
    } // Guardar las variables del plan

    Chain worst_case_chain(Pos<int> base, vector<vector<float>> base_grad, float crange, int com_type, vector<vector<float>> static_map, vector<vector<float>> communication_map)
    {
        // Seleccionar el máximo del gradiente, punto más alejado
        int x, y;
        find_matr_max_nf(base_grad, x, y);

        // Camino hasta el punto
        Path chain_path;
        chain_path.gradient_descent_(base_grad, x, y);
        // Obtener la cadena hasta ese punto
        Poss<int> sg; sg(2);
        sg.x[0] = base.x; sg.y[0] = base.y;
        sg.x[1] = x; sg.y[1] = y;

        // "chain_path.tam" para que no falten agentes a la hora de obtener las posiciones de los relays
        Chain plan = compute_chain_plan(chain_path, com_type, crange, chain_path.tam, static_map, communication_map);

        return plan;
    } // Obtener el plan al punto más alejado de todo el entorno ("el peor caso")

    vector<int> allocation(Chain &plan, Poss<int> agents, vector<vector<float>> planning_map, vector<vector<vector<float>>> &agent_grads)
    {
        vector<int> alloc;
        vector<vector<vector<float>>> goals_grads(plan.pos.x.size());
        if(plan.pos.x.size() > agents.x.size()){
            plan.unfeasible = 2;
            return alloc;
        }
        // Asignar
        if(plan.unfeasible==0){ // La cadena entera es posible
            // Formar la matriz de costes
            vector<vector<float>> costes(plan.pos.x.size(), vector<float>(agents.x.size(),0));
            for(int j=0; j<plan.pos.x.size(); j++){
                FMM gr(plan.pos.x[j], plan.pos.y[j], planning_map);
                goals_grads[j] = gr.pos_coverage(agents);
                for(int k=0; k<agents.x.size(); k++){
                    // NO ME ACUERDO PARA QUE PONÍA ESTO, SUPONGO QUE LA CAGO BASTANTE AJUSTANDO LOS MAPAS SOBRE LOS QUE PLANIFICO
                    if(!isnan(goals_grads[j][agents.x[k]][agents.y[k]]))
                        costes[j][k] = goals_grads[j][agents.x[k]][agents.y[k]];
                }
            }

            // Asignar
            hungarian a;
            a.solve(costes, alloc);

            // Almacenar los gradientes que utilizan los agentes
            for(int j=0; j<plan.pos.x.size(); j++){
                agent_grads[alloc[j]] = goals_grads[j];
                //cout<<"Gradiente para agente "<<alloc[j]<<": "<<goals_grads[j][plan.pos.x[j]][plan.pos.y[j]]<<" - "<<goals_grads[j][agents.x[alloc[j]]][agents.y[alloc[j]]]<<endl;
            }


            //cin.get();
        }
        return alloc;
    }

}

#endif