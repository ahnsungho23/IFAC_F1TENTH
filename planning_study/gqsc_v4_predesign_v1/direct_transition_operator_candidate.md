# Direct geometry transition operator candidate

## Decision: not defined

Section 5 does not establish a geometry-only operator equivalent to the legacy witness.  It
establishes a **candidate-construction plus exact-validator feedback algorithm**:

```text
entry bracket -> construct P3 -> exact validate -> steer bracket -> repeat
```

Using that mechanism in standalone GQSC would violate the task constraints by running legacy P3
construction/validation or harvesting legacy factors.  Copying the observed `0.8629-0.9128` and
`~0.577` values would instead encode data-specific outputs.  Neither is permitted.

Accordingly, candidate D is `NOT_APPLICABLE_NO_GENERAL_DERIVATION`; no operator code, transition
constant, validator call, or runtime experiment was added.  This is deliberately narrower than
claiming that a direct operator is impossible in principle.

`[INFERENCE]` A future derivation would need an explicit preconstruction model of the two active
boundaries—curvature/rate/slope below and footprint/track above—and a proof that its root preserves
the exact validator contract.  The current cheap GQSC geometry proxies do not supply that proof.
