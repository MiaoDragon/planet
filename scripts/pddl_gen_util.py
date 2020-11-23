class PDDLDefineConstruct():
    """
    fields:
    - problem
    - content
        - domain
        - objects
        - init
        - goal
    """
    def __init__(self, problem, content):
        self.problem = problem
        self.content = content
        pass
    def to_string(self):
        res = "(define " + self.problem.to_string() + "\n"
        res += self.content.to_string()
        res += ")\n"
        return res
    
class PDDLDefineContent():
    def __init__(self, domain, objects, init, goal):
        self.domain = domain
        self.objects = objects
        self.init = init
        self.goal = goal
    def to_string(self):
        res = self.domain.to_string() + "\n"
        res += self.objects.to_string() + "\n"
        res += self.init.to_string() + "\n"
        res += self.goal.to_string() + "\n"
        return res

class PDDLProblem():
    def __init__(self, problem_name):
        self.problem_name = problem_name
    def to_string(self):
        res = "(problem %s)" % (self.problem_name)
        return res

class PDDLDomain():
    def __init__(self, domain_name):
        self.domain_name = domain_name
    def to_string(self):
        res = "(:domain %s)" % (self.domain_name)
        return res

class PDDLObjects():
    """
    example:
    ============================================
    (:objects
    table1 table2 table3 table4 -surface
    lgrippertoolframe rgrippertoolframe -manipulator
    bs1 bs2 bs3 gs1 gs2 -objs )
    """
    def __init__(self, obj_dict):
        """
        object_dict: name (e.g. surface) => object_names (e.g. table1, table2, etc.)
        """
        self.obj_dict = obj_dict
        #for obj_type, obj_names in obj_dict:
    def to_string(self):
        res = "(:objects\n"
        for obj_type, obj_names in self.obj_dict.items():
            for obj_name in obj_names:
                res += obj_name + " "
            res += "-%s" % (obj_type)+'\n'
        res += ")\n"
        return res

class PDDLInit():
    """
    (:init
    (on-surface bs1 table1) (on-surface bs2 table1) (on-surface bs3 table1)
    (on-surface gs1 table1) (on-surface gs2 table1)
    (arm-empty lgrippertoolframe) (arm-empty rgrippertoolframe))
    """
    def __init__(self, pred_list):
        # pred_list: predicate list
        self.pred_list = pred_list
    def to_string(self):
        res = "(:init\n"
        for pred in self.pred_list:
            res += pred.to_string() + "\n"
        res += ")\n"
        return res

class PDDLGoal():
    """
    (:goal (and (on-surface bs1 table3) (on-surface bs2 table3) (on-surface bs3 table3) (on-surface gs1 table4) 
    (on-surface gs2 table4) (arm-empty rgrippertoolframe) (arm-empty lgrippertoolframe)))
    """
    def __init__(self, pred_list):
        self.pred_list = pred_list
    def to_string(self):
        res = "(:goal\n"
        for pred in self.pred_list:
            res += pred.to_string() + "\n"
        res += ")\n"
        return res

class PDDLPredicate():
    """
    single predicate, or nested predicate
    example:
    ===========================
    (on-surface bs1 table3)
    (and (on-surface bs1 table3) (on-surface bs2 table3))
    (arm-empty rgrippertoolframe)
    """
    def __init__(self, operator, operants):
        self.operator = operator
        self.operants = operants
    def to_string(self):
        res = "(" + self.operator.to_string() + " "
        for operant in self.operants[:-1]:
            res += operant.to_string() + " "
        res += self.operants[-1].to_string()
        res += ")"
        return res

class PDDLSymbol():
    """
    operators, operants, object name, etc.
    """
    def __init__(self, name):
        self.name = name
    def to_string(self):
        res = self.name
        return res