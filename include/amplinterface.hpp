#ifndef AMPLINTERFACE_H
#define AMPLINTERFACE_H

#include "IpUtils.hpp"
#include "IpTNLP.hpp"
#include "IpJournalist.hpp"
#include "IpOptionsList.hpp"
#include <map>
#include <string>

/* non Ipopt forward declaration */
struct ASL_pfgh;
struct SufDecl;
struct SufDesc;

using namespace Ipopt;


/** Class AmplSuffixHandler */
class AmplSuffixHandler : public ReferencedObject
{
public:
    /** Default Constructor */
  AmplSuffixHandler();

  /**  A descructor */
  ~AmplSuffixHandler();

  /** Type pf suffix */
  enum Suffix_Type
  {
    Index_Type,
    Number_Type
  };

  /** Source of the suffix */
  enum Suffix_Source
  {
    Variable_Source,
    Constraint_Source,
    Objective_Source,
    Problem_Source
  };


  /** Add the available suffix */
  void AddAvailableSuffix(std::string suffix_string, Suffix_Source source, Suffix_Type type)
  {
    suffix_ids_.push_back(suffix_string);
    suffix_types_.push_back(type);
    suffix_sources_.push_back(source);
    //      suffix_values_.push_back();
  }

  /** Get the value of the integer suffix */
  const Index* GetIntegerSuffixValues(std::string suffix_string, Suffix_Source source) const;
  /** Get the value of the number suffix */
  const Number* GetNumberSuffixValues(std::string suffix_string, Suffix_Source source) const;
  /** Get the values of the integer suffix */
  std::vector<Index> GetIntegerSuffixValues(Index n, std::string suffix_string, Suffix_Source source) const;
  /** Get the value of the number suffix */
  std::vector<Number> GetNumberSuffixValues(Index n, std::string suffix_string, Suffix_Source source) const;

private:
  /**@name Default Compiler Generated Methods
   * (Hidden to avoid implicit creation/calling).
   * These methods are not implemented and
   * we do not want the compiler to implement
   * them for us, so we declare them private
   * and do not define them. This ensures that
   * they will not be implicitly created/called. */
  //@{
  /** Default Constructor */
  //AmplSuffixHandler();

  /** Copy Constructor */
  AmplSuffixHandler(const AmplSuffixHandler&);

  /** Overloaded Equals Operator */
  void operator=(const AmplSuffixHandler&);
  //@}

  mutable ASL_pfgh* asl_; /**< asl_ */

  SufDecl* suftab_; /**< suftab_ */

  std::vector<std::string> suffix_ids_;/**< suffix_ids_ */
  std::vector<Suffix_Type> suffix_types_;/**< suffix_types_ */
  std::vector<Suffix_Source> suffix_sources_;/**< suffix_sources_ */

  /** Method called by AmplInterface to prepare the asl for the suffixes */
  void PrepareAmplForSuffixes(ASL_pfgh* asl);

  /** Method called by AmplInterface to retrieve the suffixes from asl */
  //    void RetrieveSuffixesFromAmpl(ASL_pfgh* asl);

  friend class AmplInterface;

}; // class AmplSuffixHandler

/** Class for storing a number of AMPL options that should be
 *  registered to the AMPL Solver library interface */
class AmplOptionsList : public ReferencedObject
{
public:
    /** type of the option for AMPL*/
  enum AmplOptionType {
    String_Option,
    Number_Option,
    Integer_Option,
    WS_Option,  /* this is for AMPL's internal wantsol callback */
    HaltOnError_Option /* this is for our setting of the nerror_ member */
  };

  /** Ampl Option class, contains name, type and description for an
   *  AMPL option */
class AmplOption : public ReferencedObject
  {
  public:
    /** Default Constructor */
    AmplOption(const std::string ipopt_option_name,
               AmplOptionType type,
               const std::string description);

    /** A destructor */
    ~AmplOption()
    {
      delete [] description_;
    }

    /** this method returns the option name */
    const std::string& IpoptOptionName() const
    {
      return ipopt_option_name_;
    }
    /** this method returns the type */
    AmplOptionType Type() const
    {
      return type_;
    }
    /** this method returns the description*/
    char* Description() const
    {
      return description_;
    }
  private:
    /**@name Default Compiler Generated Methods
     * (Hidden to avoid implicit creation/calling).
     * These methods are not implemented and
     * we do not want the compiler to implement
     * them for us, so we declare them private
     * and do not define them. This ensures that
     * they will not be implicitly created/called. */
    //@{
    /** Default Constructor */
    AmplOption();

    /** Copy Constructor */
    AmplOption(const AmplOption&);

    /** Overloaded Equals Operator */
    void operator=(const AmplOption&);
    //@}

    const std::string ipopt_option_name_; /**< ipopt_option_name_ */
    const AmplOptionType type_;/**< type_ */
    char* description_;/**< description_ */
  }; // class AmplOption

    /** Class PrivatInfo*/
  class PrivatInfo
  {
  public:
      /** Default Constructor */
    PrivatInfo(const std::string ipopt_name,
               SmartPtr<OptionsList> options,
               SmartPtr<const Journalist> jnlst,
               void** nerror = NULL)
        :
        ipopt_name_(ipopt_name),
        options_(options),
        jnlst_(jnlst),
        nerror_(nerror)
    {}

    /** this method returns the name */
    const std::string& IpoptName() const
    {
      return ipopt_name_;
    }
    /** this method returns the option */
    const SmartPtr<OptionsList>& Options() const
    {
      return options_;
    }
    /** this method returns the journalist */
    const SmartPtr<const Journalist>& Jnlst() const
    {
      return jnlst_;
    }

    /** this method returns the error*/
    void** NError()
    {
      return nerror_;
    }
  private:
    const std::string ipopt_name_;/**< ipopt_name_ */
    const SmartPtr<OptionsList> options_;/**< options_ */
    const SmartPtr<const Journalist> jnlst_;/**< jnlst_ */
    void** nerror_;/**< nerror_ */

  }; // class PrivatInfo

public:
  /** Default Constructor */
  AmplOptionsList()
      :
      keywds_(NULL),
      nkeywds_(0)
  {}

  /** Destructor */
  ~AmplOptionsList();

  /** Adding a new AMPL Option */
  void AddAmplOption(const std::string ampl_option_name,
                     const std::string ipopt_option_name,
                     AmplOptionsList::AmplOptionType type,
                     const std::string description)
  {
    SmartPtr<AmplOption> new_option =
      new AmplOption(ipopt_option_name, type, description);
    ampl_options_map_[ampl_option_name] = ConstPtr(new_option);
  }

  /** Number of AMPL Options */
  Index NumberOfAmplOptions()
  {
    return (Index)ampl_options_map_.size();
  }

  /** ASL keywords list for the stored options. */
  void* Keywords(const SmartPtr<OptionsList>& options,
                 SmartPtr<const Journalist> jnlst,
                 void** nerror);

private:
  /**@name Default Compiler Generated Methods
   * (Hidden to avoid implicit creation/calling).
   * These methods are not implemented and
   * we do not want the compiler to implement
   * them for us, so we declare them private
   * and do not define them. This ensures that
   * they will not be implicitly created/called. */
  //@{
  /** Default Constructor */
  //AmplOptionsList();

  /** Copy Constructor */
  AmplOptionsList(const AmplOptionsList&);

  /** Overloaded Equals Operator */
  void operator=(const AmplOptionsList&);
  //@}

  /**
   * @brief MakeValidLatexString
   * @param source
   * @param dest
   */
  void MakeValidLatexString(std::string source, std::string& dest) const;

  /**
   * @brief PrintLatex
   * @param jnlst
   */
  void PrintLatex(SmartPtr<const Journalist> jnlst);

  /** map for storing registered AMPL options */
  std::map<std::string, SmartPtr<const AmplOption> > ampl_options_map_;
  // AW: I think it should be with const like in the following line
  //     but with const the AIX compiler fails
  // std::map<const std::string, SmartPtr<const AmplOption> > ampl_options_map_;

  /** pointer to the keywords */
  void* keywds_;

  /** Number of entries stored in keywds_ */
  Index nkeywds_;

}; // class AmplOptionsList


/** AmplInterface.
 *  AmplInterface, implemented as a TNLP.
 */
class AmplInterface :  public TNLP
{

public:
    /**@name Constructors/Destructors */
    //@{
    /** Constructor. */
    AmplInterface(const SmartPtr<const Journalist>& jnlst,
             const SmartPtr<OptionsList> options,
             char*& arg, SmartPtr<AmplSuffixHandler>
             suffix_handler = NULL, bool allow_discrete = false,
             SmartPtr<AmplOptionsList> ampl_options_list = NULL,
             const char* ampl_option_string = NULL,
             const char* ampl_invokation_string = NULL,
             const char* ampl_banner_string = NULL,
             std::string* nl_file_content = NULL);

    /** Default destructor */
    virtual ~AmplInterface();
    //@}

    /** Exceptions */
    DECLARE_STD_EXCEPTION(NONPOSITIVE_SCALING_FACTOR);

    /**@name methods to gather information about the NLP. These
    * methods are overloaded from TNLP. See TNLP for their more
    * detailed documentation. */
    //@{
    /** returns dimensions of the nlp. Overloaded from TNLP */
    virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                              Index& nnz_h_lag, IndexStyleEnum& index_style);

    /** returns names and other meta data for the variables and constraints
     *  Overloaded from TNLP */
    virtual bool get_var_con_metadata(Index n,
                                      StringMetaDataMapType& var_string_md,
                                      IntegerMetaDataMapType& var_integer_md,
                                      NumericMetaDataMapType& var_numeric_md,
                                      Index m,
                                      StringMetaDataMapType& con_string_md,
                                      IntegerMetaDataMapType& con_integer_md,
                                      NumericMetaDataMapType& con_numeric_md);

    /** returns bounds of the nlp. Overloaded from TNLP */
    virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                                 Index m, Number* g_l, Number* g_u);

    /** Returns the constraint linearity.
     * array will be alocated with length n. (default implementation
     *  just return false and does not fill the array). */
    virtual bool get_constraints_linearity(Index m,
                                           LinearityType* const_types);

    /** provides a starting point for the nlp variables. Overloaded
    from TNLP */
    virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                    bool init_z, Number* z_L, Number* z_U,
                                    Index m, bool init_lambda, Number* lambda);

    /** evaluates the objective value for the nlp. Overloaded from TNLP */
    virtual bool eval_f(Index n, const Number* x, bool new_x,
                        Number& obj_value);

    /** evaluates the gradient of the objective for the
    nlp. Overloaded from TNLP */
    virtual bool eval_grad_f(Index n, const Number* x, bool new_x,
                             Number* grad_f);

    /** evaluates the constraint residuals for the nlp. Overloaded from TNLP */
    virtual bool eval_g(Index n, const Number* x, bool new_x,
                        Index m, Number* g);

    /** specifies the jacobian structure (if values is NULL) and
     *  evaluates the jacobian values (if values is not NULL) for the
     *  nlp. Overloaded from TNLP */
    virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                            Index m, Index nele_jac, Index* iRow,
                            Index *jCol, Number* values);

    /** specifies the structure of the hessian of the lagrangian (if
     *  values is NULL) and evaluates the values (if values is not
     *  NULL). Overloaded from TNLP */
    virtual bool eval_h(Index n, const Number* x, bool new_x,
                        Number obj_factor, Index m, const Number* lambda,
                        bool new_lambda, Index nele_hess, Index* iRow,
                        Index* jCol, Number* values);

    /** retrieve the scaling parameters for the variables, objective
     *  function, and constraints. */
    virtual bool get_scaling_parameters(Number& obj_scaling,
                                        bool& use_x_scaling, Index n,
                                        Number* x_scaling,
                                        bool& use_g_scaling, Index m,
                                        Number* g_scaling);
    //@}

    /** @name Solution Methods */
    //@{
    virtual void finalize_solution(SolverReturn status,
                                   Index n, const Number* x, const Number* z_L, const Number* z_U,
                                   Index m, const Number* g, const Number* lambda,
                                   Number obj_value,
                                   const IpoptData* ip_data,
                                   IpoptCalculatedQuantities* ip_cq);
    //@}

    /** @name Method for quasi-Newton approximation information. */
    //@{
    virtual Index get_number_of_nonlinear_variables();
    virtual bool get_list_of_nonlinear_variables(Index num_nonlin_vars,
        Index* pos_nonlin_vars);
    //@}


    /**@name Ampl specific methods */
    //@{
    /** Return the ampl solver object (ASL*) */
    ASL_pfgh* AmplSolverObject()
    {
      return asl_;
    }

    /** Write the solution file.  This is a wrapper for AMPL's
     *  write_sol.  TODO Maybe this should be at a different place, or
     *  collect the numbers itself? */
    void write_solution_file(const std::string& message) const;

    /** ampl orders the variables like (continuous, binary, integer).
     *  This method gives the number of binary and integer variables.
     *  For details, see Tables 3 and 4 in "Hooking Your Solver to
     *  AMPL"
     */
    void get_discrete_info(Index& nlvb_,
                           Index& nlvbi_,
                           Index& nlvc_,
                           Index& nlvci_,
                           Index& nlvo_,
                           Index& nlvoi_,
                           Index& nbv_,
                           Index& niv_) const;
    //@}

    /** A method for setting the index of the objective function to be
     *  considered.  This method must be called after the constructor,
     *  and before anything else is called.  It can only be called
     *  once, and if there is more than one objective function in the
     *  AMPL model, it MUST be called. */
    void set_active_objective(Index obj_no);

    /**@name Methods to set meta data for the variables
     * and constraints. These values will be passed on
     * to the TNLP in get_var_con_meta_data
     */
    //@{
    void set_string_metadata_for_var(std::string tag, std::vector<std::string> meta_data)
    {
      var_string_md_[tag] = meta_data;
    }

    void set_integer_metadata_for_var(std::string tag, std::vector<Index> meta_data)
    {
      var_integer_md_[tag] = meta_data;
    }

    void set_numeric_metadata_for_var(std::string tag, std::vector<Number> meta_data)
    {
      var_numeric_md_[tag] = meta_data;
    }

    void set_string_metadata_for_con(std::string tag, std::vector<std::string> meta_data)
    {
      con_string_md_[tag] = meta_data;
    }

    void set_integer_metadata_for_con(std::string tag, std::vector<Index> meta_data)
    {
      con_integer_md_[tag] = meta_data;
    }

    void set_numeric_metadata_for_con(std::string tag, std::vector<Number> meta_data)
    {
      con_numeric_md_[tag] = meta_data;
    }
    //@}

    /** Method for returning the suffix handler */
    SmartPtr<AmplSuffixHandler> get_suffix_handler()
    {
      return suffix_handler_;
    }

    /** Method to return the solutions of the problem*/
    void get_solutions(std::vector<Number>& x,
                       std::vector<Number>& z_L,
                       std::vector<Number>& z_U,
                       std::vector<Number>& lambda,
                       Number& obj);

    /** Method to return the status of the solver at the end of the optimization*/
    SolverReturn get_status();

private:
  /**@name Default Compiler Generated Methods
   * (Hidden to avoid implicit creation/calling).
   * These methods are not implemented and
   * we do not want the compiler to implement
   * them for us, so we declare them private
   * and do not define them. This ensures that
   * they will not be implicitly created/called. */
  //@{
  /** Default Constructor */
  AmplInterface();

  /** Copy Constructor */
  AmplInterface(const AmplInterface&);

  /** Overloaded Equals Operator */
  void operator=(const AmplInterface&);
  //@}

  /** Journlist */
  SmartPtr<const Journalist> jnlst_;

  /** pointer to the main ASL structure */
  ASL_pfgh* asl_;

  /** Sign of the objective fn (1 for min, -1 for max) */
  double obj_sign_;

  /**@name Problem Size Data*/
  //@{
  Index nz_h_full_; // number of nonzeros in the full_x hessian
  /* the rest of the problem size data is available easily through the ampl variables */
  //@}

  /**@name Internal copies of data */
  //@{
  /** Solution Vectors */
  Number* x_sol_;
  Number* z_L_sol_;
  Number* z_U_sol_;
  Number* g_sol_;
  Number* lambda_sol_;
  Number obj_sol_;
  //@}
  /** Status returned by the solver */
  SolverReturn status_;
  Index num_vars; /**< Number of the variables */
  Index num_cons; /**< Number of the constraints*/

  /**@name Flags to track internal state */
  //@{
  /** true when the objective value has been calculated with the
   *  current x, set to false in apply_new_x, and set to true in
   *  internal_objval */
  bool objval_called_with_current_x_;
  /** true when the constraint values have been calculated with the
   *  current x, set to false in apply_new_x, and set to true in
   *  internal_conval */
  bool conval_called_with_current_x_;
  /** true when we have called hesset */
  bool hesset_called_;
  /** true when set_active_objective has been called */
  bool set_active_objective_called_;
  //@}

  /** Pointer to the Oinfo structure */
  void* Oinfo_ptr_;

  /** nerror flag passed to ampl calls - set to NULL to halt on error */
  void* nerror_;

  /** Suffix Handler */
  SmartPtr<AmplSuffixHandler> suffix_handler_;

  /** Make the objective call to ampl */
  bool internal_objval(const Number* x, Number& obj_val);

  /** Make the constraint call to ampl*/
  bool internal_conval(const Number* x, Index m, Number* g=NULL);

  /** Internal function to update the internal and ampl state if the
   *  x value changes */
  bool apply_new_x(bool new_x, Index n, const Number* x);


  /** Method for obtaining the name of the NL file and the options
   *  set from AMPL.  Returns a pointer to a char* with the name of
   *  the stub */
  //char* get_options(const SmartPtr<OptionsList>& options,
    //                SmartPtr<AmplOptionsList>& ampl_options_list,
      //              const char* ampl_option_string,
        //            const char* ampl_invokation_string,
          //          const char* ampl_banner_string, char**& argv);



  /** returns true if the ampl nerror code is ok */
  bool nerror_ok(void* nerror);

  /** calls hesset ASL function */
  void call_hesset();

  /** meta data to pass on to TNLP */
  StringMetaDataMapType var_string_md_; /**<  variable string metadata */
  IntegerMetaDataMapType var_integer_md_; /**< variable integer metadata */
  NumericMetaDataMapType var_numeric_md_; /**< variable numeric metadata */
  StringMetaDataMapType con_string_md_; /**< constraint string metadata */
  IntegerMetaDataMapType con_integer_md_; /**< constraint integer metadata*/
  NumericMetaDataMapType con_numeric_md_; /**< constraint numeric metadata */


}; // class AmplInterface

#endif // AMPLINTERFACE_H
