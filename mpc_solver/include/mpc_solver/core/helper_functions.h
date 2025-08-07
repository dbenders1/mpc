#ifndef MPC_SOLVER_HELPER_FUNCTIONS_H
#define MPC_SOLVER_HELPER_FUNCTIONS_H

#include <sstream>
#include <string>
#include <vector>

namespace mpc_solver {

int returnDataPosition(std::string find_name,
                       std::vector<std::string> stored_names);

std::vector<int> returnDataPositions(std::vector<std::string> find_names,
                                     std::vector<std::string> stored_names);

template <class T>
void printTableColumnEntry(T name, int size, bool last);

void printTableLine(int size);

void printTableSmallLine(int size);

void printTableTitle(std::string name, int size);

void printTableTimesteps(int names_width, int enum_width, int width,
                         int n_columns);

template <class T>
void printTableInfoRow(int &iter, T *ptr_begin, int size,
                       std::vector<std::string> &members, int column_width);

void printWarning();

void printSection(double *pointer_begin, int data_size, int rows,
                  std::vector<std::string> &var_names,
                  std::vector<std::ostringstream> &streams, int names_width,
                  int value_width, int precision, int enumerate_width);

void printSectionName(int total_width, std::string name, bool first_name);

void printDataWithName(std::vector<std::ostringstream> &streams,
                       int total_width, std::string name, bool start);

void printTableInfo(int width, std::vector<std::string> &members,
                    int *ptr_solver_info);

void printTableOutput(int width, double *ptr_solver_struct,
                      std::vector<int> &sizes, int N, int precision,
                      std::vector<std::string> &sections,
                      std::vector<std::string> &var_names);

void printTableParams(int width, double *ptr_solver_x0,
                      double *ptr_solver_xinit,
                      double *ptr_solver_all_parameters,
                      std::vector<int> &sizes, int N, int precision,
                      std::vector<std::string> &sections,
                      std::vector<std::string> &var_names,
                      std::vector<std::string> &parameter_names);

void debugPrintTableVariablesString(const unsigned int &w_table,
                                    const std::string &name);

void debugPrintTableBegin(const unsigned int &w_table,
                          const unsigned int &k_start_table,
                          const unsigned int &n_stage_col,
                          const unsigned int &w_col1,
                          const unsigned int &w_coln, bool &is_last_table);

void debugPrintTableEnd(const unsigned int &w_table);

void debugPrintTableEmptyLine(const unsigned int &w_table);

void debugPrintTableEmptyColLine(const unsigned int &n_stage_col,
                                 const unsigned int &w_col1,
                                 const unsigned int &w_coln,
                                 const bool &is_idx_line);

} /*namespace mpc_solver*/
#endif