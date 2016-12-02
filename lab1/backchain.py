from production import AND, OR, NOT, PASS, FAIL, IF, THEN, \
     match, populate, simplify, variables
from zookeeper import ZOOKEEPER_RULES

# This function, which you need to write, takes in a hypothesis
# that can be determined using a set of rules, and outputs a goal
# tree of which statements it would need to test to prove that
# hypothesis. Refer to the problem set (section 2) for more
# detailed specifications and examples.

# Note that this function is supposed to be a general
# backchainer.  You should not hard-code anything that is
# specific to a particular rule set.  The backchainer will be
# tested on things other than ZOOKEEPER_RULES.


def backchain_to_goal_tree(rules, hypothesis):
    tree = OR(hypothesis)
    for rule in rules:
        for cons in rule.consequent():
            bindings = match(cons, hypothesis)
            if bindings is not None:
                child = rule.antecedent()
                if isinstance(child, list):
                    cs = [backchain_to_goal_tree(rules, populate(c, bindings)) for c in child]
                    tree.append(OR(cs) if isinstance(child, OR) else AND(cs))
                else:
                    tree.append(backchain_to_goal_tree(rules, populate(child, bindings)))
    return simplify(tree)

# Here's an example of running the backward chainer - uncomment
# it to see it work:
#print backchain_to_goal_tree(ZOOKEEPER_RULES, 'opus is a penguin')
