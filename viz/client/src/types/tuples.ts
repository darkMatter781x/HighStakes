type Grow<T, A extends Array<T>> = ((x: T, ...xs: A) => void) extends (
  ...a: infer X
) => void
  ? X
  : never;
/**
 * 
 * @note Remove `& null` to see error messages 
 */
type GrowToSize<T, A extends Array<T>, N extends number> = 1e2000 extends N
  ? "ERROR: N must be a fixed number!" & { __error: true } & never
  : A[10000000] extends undefined
  ? A["length"] extends N
    ? A
    : GrowToSize<T, Grow<T, A>, N>
  : "ERROR: A is not a tuple!" & { __error: true } & never;

type ArrayLengthMutationKeys =
  | "splice"
  | "push"
  | "pop"
  | "shift"
  | "unshift"
  | number;
type ArrayItems<T extends Array<unknown>> = T extends Array<infer TItems>
  ? TItems
  : never;

/** Removes methods that are not necessary on a fixed length array */
export type Tuple<Arr extends unknown[]> = Pick<
  Arr,
  Exclude<keyof Arr, ArrayLengthMutationKeys>
> & { [Symbol.iterator]: () => IterableIterator<ArrayItems<Arr>> };

/** Creates a fixed length array of homogenous type T with length L */
export type HomogenousTuple<
  T extends unknown,
  L extends number
> = Tuple<GrowToSize<T, [], L>>;
