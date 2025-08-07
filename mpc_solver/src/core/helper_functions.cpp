
#include <mpc_solver/core/helper_functions.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>

namespace mpc_solver {

int returnDataPosition(std::string find_name,
                       std::vector<std::string> stored_names) {
  int idx;

  // Need to find the positions we need in the solver data, but we only need to
  // do this once because it is an expensive search.
  std::vector<std::string>::iterator it_position =
      std::find(stored_names.begin(), stored_names.end(), find_name);

  if (it_position == stored_names.end()) {
    std::cerr
        << "\033[31m[MPC solver] Error during parameter position "
           "initialisation, one of the parameters ("
        << find_name
        << ") not known by the solver and the index cannot be returned! \033[0m"
        << std::endl;
    idx = -1;
  } else {
    idx = it_position - stored_names.begin();
  }

  return idx;
}

std::vector<int> returnDataPositions(std::vector<std::string> find_names,
                                     std::vector<std::string> stored_names) {
  std::vector<int> idexes(find_names.size());
  int i = 0;
  // Need to find the positions we need in the solver data, but we only need to
  // do this once because it is an expensive search.
  for (std::vector<std::string>::iterator name = find_names.begin();
       name != find_names.end(); ++name) {
    std::vector<std::string>::iterator it_position =
        std::find(stored_names.begin(), stored_names.end(), *name);

    if (it_position == stored_names.end()) {
      // ROS_ERROR_STREAM("[MPC solver] Error during parameter position
      // initialisation, one of the parameters (" << *name << ") not known by
      // the solver and the index cannot be returned!");
      std::cerr << "\033[31m[MPC solver] Error during parameter position "
                   "initialisation, one of the parameters ("
                << *name
                << ") not known by the solver and the index cannot be "
                   "returned! \033[0m"
                << std::endl;
      idexes[i] = -1;
    } else {
      idexes[i] = it_position - stored_names.begin();
    }
    ++i;
  }

  return idexes;
}

template <class T>
void printTableColumnEntry(T name, int size, bool last) {
  if (last) {
    std::cout << "|" << std::setw(size - 2) << std::right << name << "|\n";
    return;
  }

  std::cout << "|" << std::setw(size - 1) << std::right << name;
}

// Define the above function
template void printTableColumnEntry<std::string>(std::string name, int size,
                                                 bool last);
template void printTableColumnEntry<int>(int name, int size, bool last);
template void printTableColumnEntry<double>(double name, int size, bool last);

void printTableLine(int size) {
  std::cout << "|" << std::string(size - 2, '=') << "|\n";
}

void printTableSmallLine(int size) {
  std::cout << "|" << std::string(size - 2, '~') << "|\n";
}

void printTableTitle(std::string name, int size) {
  std::cout << "|" << std::string(size - 2, '=') << "|\n";
  std::cout << "|" << std::setw(size - 2) << std::left << name << "|\n";
  std::cout << "|" << std::string(size - 2, '=') << "|\n";
}

void printTableTimesteps(int names_width, int enum_width, int width,
                         int n_columns) {
  std::cout << "|" << std::setw(names_width + enum_width) << std::right << "k";
  for (int i = 0; i < n_columns; i++) {
    printTableColumnEntry(i, width, false);
  }
  std::cout << "|" << std::endl;
}

template <class T>
void printTableInfoRow(int &iter, T *ptr_begin, int size,
                       std::vector<std::string> &members, int column_width) {
  for (T *ptr = ptr_begin; ptr < ptr_begin + size; ptr++) {
    printTableColumnEntry<std::string>(members[iter], column_width, false);
    printTableColumnEntry<T>(*ptr, column_width, true);
    iter++;
  }
}

// Define the template in the above function
template void printTableInfoRow<double>(int &iter, double *ptr_begin, int size,
                                        std::vector<std::string> &members,
                                        int column_width);
template void printTableInfoRow<int>(int &iter, int *ptr_begin, int size,
                                     std::vector<std::string> &members,
                                     int column_width);

void printWarning() {
  std::cout << "IMPORTANT NOTE:\n";
  std::cout << "k = 0 => table shows calculated inputs and initial states \n";
  std::cout
      << "k = 1 => table shows calculated inputs and 1st states update \n";
  std::cout << "...\n";
  std::cout << "k = N => table shows Nth states update \n";
}

void printSection(double *pointer_begin, int data_size, int rows,
                  std::vector<std::string> &var_names,
                  std::vector<std::ostringstream> &streams, int names_width,
                  int value_width, int precision, int enumerate_width) {
  bool set_names = true;
  int i = 0;
  for (double *ptr = pointer_begin; ptr < pointer_begin + data_size; ptr++) {
    if (set_names) {
      streams[i] << std::left << "|" << std::setw(enumerate_width) << i;
      streams[i] << "|" << std::setw(names_width - 1) << std::right
                 << std::setprecision(precision) << var_names[i];
    }

    streams[i] << "|" << std::setw(value_width - 1) << std::right << *ptr;
    i++;

    if (i >= rows) {
      i = 0;
      set_names = false;
    }
  }
}

void printSectionName(int total_width, std::string name, bool first_name) {
  if (!first_name) {
    std::cout << "|" << std::string(total_width - 2, '-') << "|\n";
  }
  std::cout << "|" << std::setw(total_width - 2) << std::left << name + ":"
            << "|\n";
  std::cout << "|" << std::string(total_width - 2, '-') << "|\n";
}

void printDataWithName(std::vector<std::ostringstream> &streams,
                       int total_width, std::string name, bool start) {
  printSectionName(total_width, name, start);
  for (auto it = std::begin(streams); it != std::end(streams); ++it) {
    *it << "|\n";
    std::string temp = std::move(*it).str();
    std::cout << temp;
  }
}

void printTableParams(int width, double *ptr_solver_x0,
                      double *ptr_solver_xinit,
                      double *ptr_solver_all_parameters,
                      std::vector<int> &sizes, int N, int precision,
                      std::vector<std::string> &sections,
                      std::vector<std::string> &var_names,
                      std::vector<std::string> &parameter_names) {
  int enumerate_width = 4;
  int names_width = 24;
  int total_width = N * width + names_width + enumerate_width + 2;
  printWarning();
  printTableTitle("Solver Parameters", total_width);
  printTableTimesteps(names_width, enumerate_width, width, N);
  printTableSmallLine(total_width);

  // We want to run through data in column order, but data is printed in row
  // format So we are going to store the data in a vector with each entry a
  // string stream

  // Printing of xinit
  std::vector<std::ostringstream> streams3(sizes[4]);
  std::vector<std::string> initialized_var_names = {
      var_names.begin() + (sizes[1] + sizes[3] - sizes[4]), var_names.end()};
  printSection(ptr_solver_xinit, sizes[4], sizes[4], initialized_var_names,
               streams3, names_width, width, precision, enumerate_width);
  printDataWithName(streams3, total_width, sections[1], true);

  // Printing of the inputs and states
  std::vector<std::ostringstream> streams(sizes[0]);
  printSection(ptr_solver_x0, sizes[0] * N, sizes[0], var_names, streams,
               names_width, width, precision, enumerate_width);
  printDataWithName(streams, total_width, sections[0], false);

  // Printing of the objectives, weights, constrains
  std::vector<std::ostringstream> streams2(sizes[2]);
  printSection(ptr_solver_all_parameters, sizes[2] * N, sizes[2],
               parameter_names, streams2, names_width, width, precision,
               enumerate_width);
  printDataWithName(streams2, total_width, sections[2], false);

  printTableLine(total_width);
  std::cout << std::endl;
}

void printTableOutput(int width, double *ptr_solver_struct,
                      std::vector<int> &sizes, int N, int precision,
                      std::vector<std::string> &sections,
                      std::vector<std::string> &var_names) {
  int enumerate_width = 4;
  int names_width = 24;
  int total_width = N * width + names_width + enumerate_width + 2;
  printTableTitle("Solver Output", total_width);
  printTableTimesteps(names_width, enumerate_width, width, N);
  printTableSmallLine(total_width);

  // We want to run through data in column order, but data is printed in row
  // format So we are going to store the data in a vector with each entry a
  // string stream
  std::vector<std::ostringstream> streams(sizes[0] + sizes[1]);

  int size = (sizes[0] + sizes[1]) * (N);
  printSection(ptr_solver_struct, size, sizes[0] + sizes[1], var_names, streams,
               names_width, width, precision, enumerate_width);

  int i = 0;
  printSectionName(total_width, sections[0], true);
  for (auto it = std::begin(streams); it != std::end(streams); ++it) {
    *it << "|\n";
    std::string temp = std::move(*it).str();
    std::cout << temp;
    i++;

    if (i == sizes[0]) {
      printSectionName(total_width, sections[1], false);
    }
  }
  printTableLine(total_width);
  std::cout << std::endl;
}

void printTableInfo(int width, std::vector<std::string> &members,
                    int *ptr_solver_info) {
  printTableTitle("Solver Info", width);
  int column_width = width / 2;

  // Dealing with int32 (4 bytes) and double (8 bytes) in info struct
  // We switch between 4 bytes and 8 bytes pointer when needed
  // c++ standard float is 4 bytes and double is 8 bytes
  int *pointer_int = ptr_solver_info;
  int i = 0;
  printTableInfoRow<int>(i, pointer_int, 2, members, column_width);

  double *pointer_double = (double *)(pointer_int + 2);
  printTableInfoRow<double>(i, pointer_double, 11, members, column_width);

  pointer_int = (int *)(pointer_double + 11);
  printTableInfoRow<int>(i, pointer_int, 2, members, column_width);

  pointer_double = (double *)(pointer_int + 2);
  printTableInfoRow<double>(i, pointer_double, 4, members, column_width);

  printTableLine(width);
  std::cout << std::endl;
}

void debugPrintTableVariablesString(const unsigned int &w_table,
                                    const std::string &name) {
  std::cout << "|" << std::string(w_table - 2, '-') << "|" << std::endl;
  unsigned int str_len = name.length();
  std::cout << "| " << name << std::string(w_table - str_len - 3, ' ') << "|"
            << std::endl;
  std::cout << "|" << std::string(w_table - 2, '-') << "|" << std::endl;
}

void debugPrintTableBegin(const unsigned int &w_table,
                          const unsigned int &k_start_table,
                          const unsigned int &n_stage_col,
                          const unsigned int &w_col1,
                          const unsigned int &w_coln, bool &is_last_table) {
  std::cout << std::string(w_table, '_') << std::endl;
  std::cout << "|" << std::string(w_col1, ' ');
  for (unsigned int k = k_start_table; k < k_start_table + n_stage_col; k++) {
    std::cout << "|" << std::string(w_coln - 8, ' ');
    if (k < 10)
      std::cout << "  ";
    else if (k < 100)
      std::cout << " ";
    std::cout << "k = " << k << " ";
  }
  std::cout << "|" << std::endl;
  std::cout << "|" << std::string(w_table - 2, '=') << "|" << std::endl;

  // Indicate start and end of plan
  if (k_start_table == 0) {
    std::cout << "|" << std::string(w_col1 + 1 + w_coln, '=')
              << "| Start of Plan ";
    if (!is_last_table) {
      std::cout << std::string(w_table - 1 - w_col1 - 1 - w_coln - 16 - 2, '.')
                << " |" << std::endl;
    } else {
      std::cout << std::string(
                       w_table - 1 - w_col1 - 1 - w_coln - 16 - 14 - w_coln - 1,
                       '.')
                << " end of Plan |" << std::string(w_coln, '=') << "|"
                << std::endl;
      return;
    }
  } else if (!is_last_table) {
    std::cout << "| " << std::string(w_table - 4, '.') << " |" << std::endl;
  }
  if (is_last_table) {
    std::cout << "| " << std::string(w_table - 2 - 14 - w_coln - 1, '.')
              << " end of Plan |" << std::string(w_coln, '=') << "|"
              << std::endl;
  }
}

void debugPrintTableEnd(const unsigned int &w_table) {
  std::cout << "|" << std::string(w_table - 2, '_') << "|" << std::endl;
}

void debugPrintTableEmptyLine(const unsigned int &w_table) {
  std::cout << "|" << std::string(w_table - 2, ' ') << "|" << std::endl;
}

void debugPrintTableEmptyColLine(const unsigned int &n_stage_col,
                                 const unsigned int &w_col1,
                                 const unsigned int &w_coln,
                                 const bool &is_idx_line) {
  for (unsigned int k = 0; k < n_stage_col + 1; k++) {
    if (k == 0) {
      if (is_idx_line)
        std::cout << "|" << std::string(w_col1 - 4, ' ') << "idx ";
      else
        std::cout << "|" << std::string(w_col1, ' ');
    } else if (k < n_stage_col)
      std::cout << "|" << std::string(w_coln, ' ');
    else
      std::cout << "|" << std::string(w_coln, ' ') << "|" << std::endl;
  }
}

} /*namespace mpc_solver*/
